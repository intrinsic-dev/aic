/*
 * Copyright (C) 2025 Intrinsic Innovation LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "aic_scoring/ScoringTier2.hh"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace aic_scoring {
//////////////////////////////////////////////////
ScoringTier2::ScoringTier2(rclcpp::Node *_node) : node(_node) {}

//////////////////////////////////////////////////
// TODO(luca) consider having a make function that returns a pointer which is
// nullptr if initialization failed instead.
bool ScoringTier2::Initialize(YAML::Node _config) {
  if (!this->node) {
    std::cerr << "[ScoringTier2]: null ROS node. Aborting." << std::endl;
    return false;
  }
  if (!this->ParseStats(_config)) return false;

  const rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  // Subscribe to all topics relevant for scoring.
  for (const auto &topic : this->topics) {
    auto sub = this->node->create_generic_subscription(
        topic.name, topic.type, reliable_qos,
        [this, topic](std::shared_ptr<const rclcpp::SerializedMessage> msg,
                      const rclcpp::MessageInfo &msg_info) {
          // Bag the data.
          const auto &rmw_info = msg_info.get_rmw_message_info();
          std::lock_guard<std::mutex> lock(this->mutex);
          if (this->state != State::Recording) return;

          if (topic.name == kTfStaticTopic) {
            PoseMsg efPosition;
            if (this->EndEffectorPose(efPosition)) {
              // Log the end effector position.
              auto serialized = std::make_shared<rclcpp::SerializedMessage>();
              rclcpp::Serialization<PoseMsg> serializer;
              serializer.serialize_message(&efPosition, serialized.get());

              this->bagWriter.write(serialized, kEndEffectorTopic,
                                    "geometry_msgs/msg/PoseStamped",
                                    rmw_info.received_timestamp,
                                    rmw_info.source_timestamp);
            }
          }

          this->bagWriter.write(msg, topic.name, topic.type,
                                rmw_info.received_timestamp,
                                rmw_info.source_timestamp);
        });
    this->subscriptions.push_back(sub);
  }
  return true;
}

//////////////////////////////////////////////////
void ScoringTier2::ResetConnections(
    const std::vector<Connection> &_connections) {
  this->connections = _connections;

  // Debug output.
  // std::cout << "Connections" << std::endl;
  // for (const Connection &c : this->connections)
  // {
  //   std::cout << "  plug: " << c.plugName << std::endl;
  //   std::cout << "  port: " << c.portName << std::endl;
  //   std::cout << "  Dist: " << c.distance << std::endl;
  // }
}

//////////////////////////////////////////////////
void ScoringTier2::SetGripperFrame(
    const std::string &_gripperFrame,
    std::shared_ptr<tf2_ros::Buffer> &_tfBuffer) {
  this->gripperFrame = _gripperFrame;
  this->tfBuffer = _tfBuffer;
}

//////////////////////////////////////////////////
bool ScoringTier2::StartRecording(const std::string &_filename) {
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->state != State::Idle) {
    RCLCPP_ERROR(this->node->get_logger(), "Scoring system is busy.");
    return false;
  }

  try {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = _filename;
    this->bagWriter.open(storage_options);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->node->get_logger(), "Failed to open bag: %s", e.what());
    return false;
  }
  this->state = State::Recording;
  this->bagUri = _filename;
  return true;
}

//////////////////////////////////////////////////
bool ScoringTier2::StopRecording() {
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->state != State::Recording) {
    RCLCPP_ERROR(this->node->get_logger(), "Scoring system is not recording");
    return false;
  }
  this->bagWriter.close();
  this->state = State::Idle;
  return true;
}

//////////////////////////////////////////////////
template <typename Msg>
Msg deserialize_from_rosbag(
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg_in) {
  Msg msg;
  rclcpp::SerializedMessage extracted_serialized_msg(*msg_in->serialized_data);
  rclcpp::Serialization<Msg> serialization;
  serialization.deserialize_message(&extracted_serialized_msg, &msg);
  return msg;
}

//////////////////////////////////////////////////
std::pair<Tier2Score, Tier3Score> ScoringTier2::ComputeScore() {
  // TODO(luca) actually compute score
  Tier2Score tier2_score("Scoring failed.");
  Tier3Score tier3_score(0);
  if (this->state != State::Idle) {
    RCLCPP_ERROR(this->node->get_logger(), "Scoring system is busy.");
    return {tier2_score, tier3_score};
  }
  rosbag2_cpp::Reader bagReader;

  try {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = this->bagUri;
    bagReader.open(storage_options);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->node->get_logger(), "Failed to open bag: %s", e.what());
    return {tier2_score, tier3_score};
  }
  this->state = State::Scoring;

  // Reset scoring state from previous sessions.
  this->ResetJerk();

  tier2_score.message = "Scoring succeeded.";

  while (bagReader.has_next()) {
    const auto msg_ptr = bagReader.read_next();
    // Debugging to make sure messages are in the bag
    // RCLCPP_INFO(this->node->get_logger(), "Received message on topic '%s'",
    //     msg_ptr->topic_name.c_str());
    if (msg_ptr->topic_name == kJointStateTopic) {
      const auto msg = deserialize_from_rosbag<JointStateMsg>(msg_ptr);
      this->JointStateCallback(msg);
    } else if (msg_ptr->topic_name == kEndEffectorTopic) {
      const auto msg = deserialize_from_rosbag<PoseMsg>(msg_ptr);
      this->JerkCallback(msg);
    } else if (msg_ptr->topic_name == kTfTopic) {
      const auto msg = deserialize_from_rosbag<TFMsg>(msg_ptr);
      this->TfCallback(msg);
    } else if (msg_ptr->topic_name == kTfStaticTopic) {
      const auto msg = deserialize_from_rosbag<TFMsg>(msg_ptr);
      this->TfStaticCallback(msg);
    } else if (msg_ptr->topic_name == kContactsTopic) {
      const auto msg = deserialize_from_rosbag<ContactsMsg>(msg_ptr);
      this->ContactsCallback(msg);
    } else if (msg_ptr->topic_name == kWrenchTopic) {
      const auto msg = deserialize_from_rosbag<WrenchMsg>(msg_ptr);
      this->WrenchCallback(msg);
    } else if (msg_ptr->topic_name == kMotionUpdateTopic) {
      const auto msg = deserialize_from_rosbag<MotionUpdateMsg>(msg_ptr);
      this->MotionUpdateCallback(msg);
    } else if (msg_ptr->topic_name == kJointMotionUpdateTopic) {
      const auto msg = deserialize_from_rosbag<JointMotionUpdateMsg>(msg_ptr);
      this->JointMotionUpdateCallback(msg);
    } else {
      RCLCPP_WARN(this->node->get_logger(),
                  "Unexpected topic name while scoring: %s",
                  msg_ptr->topic_name.c_str());
    }
  }

  this->state = State::Idle;
  tier2_score.add_category_score("dummy_category", 3, "It works!");
  tier3_score = Tier3Score(1);
  return {tier2_score, tier3_score};
}

//////////////////////////////////////////////////
bool ScoringTier2::ParseStats(YAML::Node _config) {
  // Parse topics to subscribe to.
  if (!_config["topics"]) {
    RCLCPP_ERROR(this->node->get_logger(),
                 "Unable to find [topics] in yaml file");
    return false;
  }

  const auto &topics = _config["topics"];
  if (!topics.IsSequence()) {
    RCLCPP_ERROR(this->node->get_logger(),
                 "Unable to find sequence of topics within [topics]");
    return false;
  }

  for (const auto &newTopic : topics) {
    if (!newTopic["topic"]) {
      RCLCPP_ERROR(this->node->get_logger(),
                   "Unrecognized element. It should be [topic]");
      return false;
    }

    const auto &topicProperties = newTopic["topic"];
    if (!topicProperties.IsMap()) {
      RCLCPP_ERROR(this->node->get_logger(),
                   "Unable to find properties within [topic]");
      return false;
    }

    TopicInfo topicInfo;

    if (!topicProperties["name"]) {
      RCLCPP_ERROR(this->node->get_logger(),
                   "Unable to find [name] within [topic]");
      return false;
    }
    topicInfo.name = topicProperties["name"].as<std::string>();

    if (!topicProperties["type"]) {
      RCLCPP_ERROR(this->node->get_logger(),
                   "Unable to find [type] within [topic]");
      return false;
    }
    topicInfo.type = topicProperties["type"].as<std::string>();

    this->topics.push_back(topicInfo);
  }

  return true;
}

//////////////////////////////////////////////////
std::set<std::string> ScoringTier2::GetMissingRequiredTopics() const {
  std::set<std::string> unavailable;
  for (const auto &subscription : this->subscriptions) {
    if (subscription->get_publisher_count() == 0) {
      unavailable.insert(subscription->get_topic_name());
    }
  }
  return unavailable;
}

//////////////////////////////////////////////////
void ScoringTier2::JointStateCallback(const JointStateMsg &_msg) { (void)_msg; }

//////////////////////////////////////////////////
void ScoringTier2::TfCallback(const TFMsg &_msg) { (void)_msg; }

//////////////////////////////////////////////////
void ScoringTier2::TfStaticCallback(const TFMsg &_msg) { (void)_msg; }

//////////////////////////////////////////////////
void ScoringTier2::ContactsCallback(const ContactsMsg &_msg) { (void)_msg; }

//////////////////////////////////////////////////
void ScoringTier2::WrenchCallback(const WrenchMsg &_msg) { (void)_msg; }

//////////////////////////////////////////////////
void ScoringTier2::MotionUpdateCallback(const MotionUpdateMsg &_msg) {
  (void)_msg;
}

//////////////////////////////////////////////////
void ScoringTier2::JointMotionUpdateCallback(const JointMotionUpdateMsg &_msg) {
  (void)_msg;
}

//////////////////////////////////////////////////
void ScoringTier2::JerkCallback(const PoseMsg &_pose) {
  // Debug output
  // std::cout << "(" << _pose.pose.position.x << " " << _pose.pose.position.y
  //           << " " << _pose.pose.position.z << ")" << std::endl;

  // Helper to convert ROS time to seconds.
  auto toSeconds = [](const builtin_interfaces::msg::Time &t) {
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
  };

  // Check timestamp is increasing.
  if (!this->poseHistory.empty()) {
    double lastTime = toSeconds(this->poseHistory.back().header.stamp);
    double newTime = toSeconds(_pose.header.stamp);
    if (newTime <= lastTime) {
      return;
    }
  }

  // Add pose to history.
  this->poseHistory.push_back(_pose);

  // Need 4 samples to compute jerk.
  if (this->poseHistory.size() < 4) {
    return;
  }

  // Keep only last 4 samples.
  if (this->poseHistory.size() > 4) {
    this->poseHistory.erase(this->poseHistory.begin());
  }

  // Extract timestamps.
  double t0 = toSeconds(this->poseHistory[0].header.stamp);
  double t1 = toSeconds(this->poseHistory[1].header.stamp);
  double t2 = toSeconds(this->poseHistory[2].header.stamp);
  double t3 = toSeconds(this->poseHistory[3].header.stamp);

  // Extract positions.
  double px[4], py[4], pz[4];
  for (int i = 0; i < 4; ++i) {
    px[i] = this->poseHistory[i].pose.position.x;
    py[i] = this->poseHistory[i].pose.position.y;
    pz[i] = this->poseHistory[i].pose.position.z;
  }

  // Compute finite differences for jerk.
  // v1 = (p1 - p0) / (t1 - t0), etc.
  // a1 = (v2 - v1) / (midpoint difference)
  // jerk = (a2 - a1) / (midpoint difference)
  auto computeJerk = [&](const double p[4]) {
    double v1 = (p[1] - p[0]) / (t1 - t0);
    double v2 = (p[2] - p[1]) / (t2 - t1);
    double v3 = (p[3] - p[2]) / (t3 - t2);

    double mid_v1 = (t0 + t1) / 2.0;
    double mid_v2 = (t1 + t2) / 2.0;
    double mid_v3 = (t2 + t3) / 2.0;

    double a1 = (v2 - v1) / (mid_v2 - mid_v1);
    double a2 = (v3 - v2) / (mid_v3 - mid_v2);

    double mid_a1 = (mid_v1 + mid_v2) / 2.0;
    double mid_a2 = (mid_v2 + mid_v3) / 2.0;

    return (a2 - a1) / (mid_a2 - mid_a1);
  };

  // Compute linear jerk.
  this->linearJerk.x = computeJerk(px);
  this->linearJerk.y = computeJerk(py);
  this->linearJerk.z = computeJerk(pz);

  // Update time-weighted average jerk.
  // Use the time interval from t1 to t2 as the weight for this jerk sample.
  double dt = t2 - t1;
  this->totalJerkTime += dt;

  // Accumulate weighted jerk.
  this->accumLinearJerk.x += this->linearJerk.x * dt;
  this->accumLinearJerk.y += this->linearJerk.y * dt;
  this->accumLinearJerk.z += this->linearJerk.z * dt;

  // Compute averages.
  if (this->totalJerkTime > 0.0) {
    this->avgLinearJerk.x = this->accumLinearJerk.x / this->totalJerkTime;
    this->avgLinearJerk.y = this->accumLinearJerk.y / this->totalJerkTime;
    this->avgLinearJerk.z = this->accumLinearJerk.z / this->totalJerkTime;
  }

  return;
}

//////////////////////////////////////////////////
bool ScoringTier2::EndEffectorPose(PoseMsg &_pose) {
  // Sanity check.
  if (this->gripperFrame.empty() || !this->tfBuffer) {
    RCLCPP_WARN(this->node->get_logger(),
                "Unable to compute end effector pose yet");
    return false;
  }

  std::string warning_msg;
  if (!this->tfBuffer->canTransform("world", this->gripperFrame,
                                    tf2::TimePointZero, &warning_msg)) {
    RCLCPP_WARN(this->node->get_logger(), "TF Wait Failed: %s",
                warning_msg.c_str());
    return false;
  }

  geometry_msgs::msg::TransformStamped t = this->tfBuffer->lookupTransform(
      "world", this->gripperFrame, tf2::TimePointZero);

  _pose.header = t.header;
  _pose.pose.position.x = t.transform.translation.x;
  _pose.pose.position.y = t.transform.translation.y;
  _pose.pose.position.z = t.transform.translation.z;
  _pose.pose.orientation = t.transform.rotation;
  return true;
}

//////////////////////////////////////////////////
ScoringTier2Node::ScoringTier2Node(const std::string &_yamlFile)
    : Node("score_tier2_node") {
  try {
    auto config = YAML::LoadFile(_yamlFile);
    this->score = std::make_unique<aic_scoring::ScoringTier2>(this);
    this->score->Initialize(config);
  } catch (const YAML::BadFile &_e) {
    std::cerr << "Unable to open YAML file [" << _yamlFile << "]" << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
ScoringTier2::Vector3Msg ScoringTier2::GetLinearJerk() const {
  return this->linearJerk;
}

//////////////////////////////////////////////////
ScoringTier2::Vector3Msg ScoringTier2::GetAvgLinearJerk() const {
  return this->avgLinearJerk;
}

//////////////////////////////////////////////////
void ScoringTier2::ResetJerk() {
  this->poseHistory.clear();
  this->linearJerk = Vector3Msg();
  this->avgLinearJerk = Vector3Msg();
  this->accumLinearJerk = Vector3Msg();
  this->totalJerkTime = 0.0;
}

}  // namespace aic_scoring
