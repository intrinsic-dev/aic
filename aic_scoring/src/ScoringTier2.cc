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
#include <sstream>
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
          if (this->state == State::Recording) {
            this->bagWriter.write(msg, topic.name, topic.type,
                                  rmw_info.received_timestamp,
                                  rmw_info.source_timestamp);
          }
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
void ScoringTier2::SetGripperFrame(const std::string &_gripperFrame) {
  this->gripperFrame = _gripperFrame;
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
  Tier3Score tier3_score(0, "Task execution failed.");
  tf2_buffer.clear();
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
  this->timestamps.clear();

  tier2_score.message = "Scoring succeeded.";

  // First pass: Process all messages to build the complete TF buffer.
  // We need both static TF (robot URDF) and dynamic TF (joint states) to
  // compute the full transform chain to the gripper.
  while (bagReader.has_next()) {
    // Debugging to make sure messages are in the bag
    // RCLCPP_INFO(this->node->get_logger(), "Received message on topic '%s'",
    //     msg_ptr->topic_name.c_str());
    const auto msg_ptr = bagReader.read_next();
    if (msg_ptr->topic_name == kJointStateTopic) {
      const auto msg = deserialize_from_rosbag<JointStateMsg>(msg_ptr);
      this->JointStateCallback(msg);
    } else if (msg_ptr->topic_name == kTfTopic ||
               msg_ptr->topic_name == kScoringTfTopic) {
      const auto msg = deserialize_from_rosbag<TFMsg>(msg_ptr);
      this->TfCallback(msg);
    } else if (msg_ptr->topic_name == kTfStaticTopic ||
               msg_ptr->topic_name == kScoringTfStaticTopic) {
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

  // Second pass: Compute jerk for each stored timestamp.
  // Now the TF buffer has the complete transform tree.
  for (const auto &t : this->timestamps) {
    auto pose = this->EndEffectorPose(t);
    if (pose.has_value()) {
      this->JerkCallback(*pose);
    }
  }

  this->state = State::Idle;
  tier3_score = this->GetDistanceScore();
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
void ScoringTier2::SetTaskStartTime(const rclcpp::Time &_time) {
  this->task_start_time = _time;
}

//////////////////////////////////////////////////
void ScoringTier2::SetTaskEndTime(const rclcpp::Time &_time) {
  this->task_end_time = _time;
}

//////////////////////////////////////////////////
void ScoringTier2::JointStateCallback(const JointStateMsg &_msg) { (void)_msg; }

//////////////////////////////////////////////////
void ScoringTier2::TfCallback(const TFMsg &_msg) {
  for (const auto &tf : _msg.transforms) {
    this->tf2_buffer.setTransform(tf, "scoring", false);
    // A bit redundant since all the messages will likely have the same
    // timestamp
    this->timestamps.insert(tf2::getTimestamp(tf));
  }
}

//////////////////////////////////////////////////
void ScoringTier2::TfStaticCallback(const TFMsg &_msg) {
  for (const auto &tf : _msg.transforms) {
    this->tf2_buffer.setTransform(tf, "scoring", true);
  }
}

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
std::optional<double> ScoringTier2::GetPlugPortDistance(
    tf2::TimePoint t) const {
  if (this->connections.empty()) {
    RCLCPP_ERROR(this->node->get_logger(), "No connection was found");
    return std::nullopt;
  }
  const auto &plug = this->connections[0].plugName;
  const auto &port = this->connections[0].portName;
  // For now we only calculate the distance for the first connection
  if (!this->tf2_buffer.canTransform("aic_world", plug, t)) {
    RCLCPP_ERROR(this->node->get_logger(), "Plug %s not found in the tf tree",
                 plug.c_str());
    return std::nullopt;
  }
  if (!this->tf2_buffer.canTransform("aic_world", port, t)) {
    RCLCPP_ERROR(this->node->get_logger(), "Port %s not found in the tf tree",
                 port.c_str());
    return std::nullopt;
  }

  const auto plug_tf = this->tf2_buffer.lookupTransform("aic_world", plug, t);
  const auto port_tf = this->tf2_buffer.lookupTransform("aic_world", port, t);

  return std::sqrt(
      (plug_tf.transform.translation.x - port_tf.transform.translation.x) *
          (plug_tf.transform.translation.x - port_tf.transform.translation.x) +
      (plug_tf.transform.translation.y - port_tf.transform.translation.y) *
          (plug_tf.transform.translation.y - port_tf.transform.translation.y) +
      (plug_tf.transform.translation.z - port_tf.transform.translation.z) *
          (plug_tf.transform.translation.z - port_tf.transform.translation.z));
}

//////////////////////////////////////////////////
// Calculates an inverse proportional score clamped to max_score for min_range
// and min_score for max_range, and with a linear inverse proportional
// interpolation inbetween.
static double CalculateInverseProportionalScore(const double max_score,
                                                const double min_score,
                                                const double max_range,
                                                const double min_range,
                                                const double measurement) {
  if (measurement >= max_range) {
    return min_score;
  } else if (measurement <= min_range) {
    return max_score;
  }

  return min_score + ((max_range - measurement) / (max_range - min_range)) *
                         (max_score - min_score);
}

//////////////////////////////////////////////////
Tier3Score ScoringTier2::GetDistanceScore() const {
  // For now, just have the score be inversely proportional to the time
  // it took to execute the task and the final distance between plug and port
  // Linear interpolation in the interval, clamp to maximum and minimum
  // Use distance as a base score, task time as a multiplier
  const rclcpp::Duration kMaxTaskTime = rclcpp::Duration::from_seconds(60.0);
  const rclcpp::Duration kMinTaskTime = rclcpp::Duration::from_seconds(5.0);
  const double kFastestTaskMultiplier = 3.0;
  const double kSlowestTaskMultiplier = 1.0;

  const double kMaxDistance = 1.0;
  const double kMinDistance = 0.0;
  const double kClosestTaskScore = 10.0;
  const double kFurthestTaskScore = 0.5;

  if (this->timestamps.empty()) {
    return Tier3Score(0, "Distance computation failed, no tfs received");
  }

  if (!this->task_start_time.has_value()) {
    return Tier3Score(0, "Time computation failed, task start time not set");
  }

  if (!this->task_end_time.has_value()) {
    return Tier3Score(0, "Time computation failed, task end time not set");
  }

  // TODO(luca) enable this when aic_engine is set to use simulation time
  // const auto end_time =
  //    std::chrono::nanoseconds(this->task_end_time.value().nanoseconds());
  // const auto dist = this->GetPlugPortDistance(tf2::TimePoint(end_time));
  const auto dist = this->GetPlugPortDistance(tf2::TimePointZero);
  if (!dist.has_value()) {
    return Tier3Score(
        0, "Distance computation failed, tf between cable and port not found");
  }

  const rclcpp::Duration task_duration =
      this->task_end_time.value() - this->task_start_time.value();
  const double duration_multiplier = CalculateInverseProportionalScore(
      kFastestTaskMultiplier, kSlowestTaskMultiplier, kMaxTaskTime.seconds(),
      kMinTaskTime.seconds(), task_duration.seconds());

  const double score =
      duration_multiplier * CalculateInverseProportionalScore(
                                kClosestTaskScore, kFurthestTaskScore,
                                kMaxDistance, kMinDistance, dist.value());

  std::stringstream sstream;
  sstream.setf(std::ios::fixed);
  sstream.precision(2);
  sstream << "Task completed in " << task_duration.seconds()
          << " seconds, with a distance of " << dist.value() << " meters";

  return Tier3Score(score, sstream.str());
}

//////////////////////////////////////////////////
void ScoringTier2::JerkCallback(const PoseMsg &_pose) {
  // Debug output
  std::cout << "(" << _pose.pose.position.x << " " << _pose.pose.position.y
            << " " << _pose.pose.position.z << ")" << std::endl;

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
}

//////////////////////////////////////////////////
std::optional<ScoringTier2::PoseMsg> ScoringTier2::EndEffectorPose(
    tf2::TimePoint t) const {
  // Sanity check.
  if (this->gripperFrame.empty()) {
    RCLCPP_ERROR(this->node->get_logger(),
                 "Unable to compute end effector pose yet");
    return std::nullopt;
  }

  std::string warning_msg;
  if (!this->tf2_buffer.canTransform("aic_world", this->gripperFrame, t)) {
    RCLCPP_ERROR(this->node->get_logger(),
                 "Gripper %s not found in the tf tree",
                 this->gripperFrame.c_str());
    return std::nullopt;
  }

  const auto gripper_tf =
      this->tf2_buffer.lookupTransform("aic_world", this->gripperFrame, t);

  ScoringTier2::PoseMsg pose;
  pose.header = gripper_tf.header;
  pose.pose.position.x = gripper_tf.transform.translation.x;
  pose.pose.position.y = gripper_tf.transform.translation.y;
  pose.pose.position.z = gripper_tf.transform.translation.z;
  pose.pose.orientation = gripper_tf.transform.rotation;
  return pose;
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
