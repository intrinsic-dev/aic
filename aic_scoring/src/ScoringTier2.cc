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
#include <rosbag2_cpp/writer.hpp>
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

  // Subscribe to all topics relevant for scoring.
  for (const auto &topic : this->topics) {
    auto sub = this->node->create_generic_subscription(
        topic.name, topic.type, rclcpp::QoS(10),
        [this, topic](std::shared_ptr<const rclcpp::SerializedMessage> msg,
                      const rclcpp::MessageInfo &msg_info) {
          // Bag the data.
          const auto &rmw_info = msg_info.get_rmw_message_info();
          std::lock_guard<std::mutex> lock(this->mutex);
          if (this->bagOpen) {
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
bool ScoringTier2::StartRecording(const std::string &_filename) {
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->bagOpen) {
    RCLCPP_ERROR(this->node->get_logger(), "Bag already opened.");
    return false;
  }

  try {
    this->bagWriter.open(_filename);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->node->get_logger(), "Failed to open bag: %s", e.what());
    return false;
  }
  this->bagOpen = true;
  return true;
}

//////////////////////////////////////////////////
bool ScoringTier2::StopRecording() {
  std::lock_guard<std::mutex> lock(this->mutex);
  if (!this->bagOpen) {
    RCLCPP_ERROR(this->node->get_logger(), "Bag already closed.");
    return false;
  }
  this->bagWriter.close();
  this->bagOpen = false;
  return true;
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
bool ScoringTier2::UpdateJerk(const geometry_msgs::msg::PoseStamped &_pose) {
  // Helper to convert ROS time to seconds.
  auto toSeconds = [](const builtin_interfaces::msg::Time &t) {
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
  };

  // Check timestamp is increasing.
  if (!this->poseHistory.empty()) {
    double lastTime = toSeconds(this->poseHistory.back().header.stamp);
    double newTime = toSeconds(_pose.header.stamp);
    if (newTime <= lastTime) {
      return false;
    }
  }

  // Add pose to history.
  this->poseHistory.push_back(_pose);

  // Need 4 samples to compute jerk.
  if (this->poseHistory.size() < 4) {
    return true;
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

  // Helper to convert quaternion to Euler angles (roll, pitch, yaw).
  auto quatToEuler = [](const geometry_msgs::msg::Quaternion &q, double &roll,
                        double &pitch, double &yaw) {
    tf2::Quaternion quat_tf;
    tf2::fromMsg(q, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
  };

  // Extract positions.
  double px[4], py[4], pz[4];
  for (int i = 0; i < 4; ++i) {
    px[i] = this->poseHistory[i].pose.position.x;
    py[i] = this->poseHistory[i].pose.position.y;
    pz[i] = this->poseHistory[i].pose.position.z;
  }

  // Extract Euler angles.
  double roll[4], pitch[4], yaw[4];
  for (int i = 0; i < 4; ++i) {
    quatToEuler(this->poseHistory[i].pose.orientation, roll[i], pitch[i],
                yaw[i]);
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

  // Compute angular jerk.
  this->angularJerk.x = computeJerk(roll);
  this->angularJerk.y = computeJerk(pitch);
  this->angularJerk.z = computeJerk(yaw);

  // Update time-weighted average jerk.
  // Use the time interval from t1 to t2 as the weight for this jerk sample.
  double dt = t2 - t1;
  this->totalJerkTime += dt;

  // Accumulate weighted jerk.
  this->accumLinearJerk.x += this->linearJerk.x * dt;
  this->accumLinearJerk.y += this->linearJerk.y * dt;
  this->accumLinearJerk.z += this->linearJerk.z * dt;

  this->accumAngularJerk.x += this->angularJerk.x * dt;
  this->accumAngularJerk.y += this->angularJerk.y * dt;
  this->accumAngularJerk.z += this->angularJerk.z * dt;

  // Compute averages.
  if (this->totalJerkTime > 0.0) {
    this->avgLinearJerk.x = this->accumLinearJerk.x / this->totalJerkTime;
    this->avgLinearJerk.y = this->accumLinearJerk.y / this->totalJerkTime;
    this->avgLinearJerk.z = this->accumLinearJerk.z / this->totalJerkTime;

    this->avgAngularJerk.x = this->accumAngularJerk.x / this->totalJerkTime;
    this->avgAngularJerk.y = this->accumAngularJerk.y / this->totalJerkTime;
    this->avgAngularJerk.z = this->accumAngularJerk.z / this->totalJerkTime;
  }

  return true;
}

//////////////////////////////////////////////////
geometry_msgs::msg::Vector3 ScoringTier2::GetLinearJerk() const {
  return this->linearJerk;
}

//////////////////////////////////////////////////
geometry_msgs::msg::Vector3 ScoringTier2::GetAngularJerk() const {
  return this->angularJerk;
}

//////////////////////////////////////////////////
geometry_msgs::msg::Vector3 ScoringTier2::GetAvgLinearJerk() const {
  return this->avgLinearJerk;
}

//////////////////////////////////////////////////
geometry_msgs::msg::Vector3 ScoringTier2::GetAvgAngularJerk() const {
  return this->avgAngularJerk;
}

//////////////////////////////////////////////////
void ScoringTier2::ResetJerk() {
  this->poseHistory.clear();
  this->linearJerk = geometry_msgs::msg::Vector3();
  this->angularJerk = geometry_msgs::msg::Vector3();
  this->avgLinearJerk = geometry_msgs::msg::Vector3();
  this->avgAngularJerk = geometry_msgs::msg::Vector3();
  this->accumLinearJerk = geometry_msgs::msg::Vector3();
  this->accumAngularJerk = geometry_msgs::msg::Vector3();
  this->totalJerkTime = 0.0;
}

//////////////////////////////////////////////////
bool ScoringTier2::UpdatePlugPortDistance(
    const geometry_msgs::msg::PointStamped &_plug,
    const geometry_msgs::msg::PointStamped &_port) {
  // Helper to convert ROS time to seconds.
  auto toSeconds = [](const builtin_interfaces::msg::Time &t) {
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
  };

  // Use plug timestamp as reference.
  double newTime = toSeconds(_plug.header.stamp);

  // Compute Euclidean distance.
  double dx = _plug.point.x - _port.point.x;
  double dy = _plug.point.y - _port.point.y;
  double dz = _plug.point.z - _port.point.z;
  double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

  // First sample: store timestamp and distance, return true.
  if (this->lastPlugPortStamp < 0) {
    this->lastPlugPortStamp = newTime;
    this->plugPortDistance = distance;
    return true;
  }

  // Check that new timestamp > last timestamp.
  if (newTime <= this->lastPlugPortStamp) {
    return false;
  }

  // Compute dt.
  double dt = newTime - this->lastPlugPortStamp;

  // Update accumulated weighted distance.
  this->accumPlugPortDistance += this->plugPortDistance * dt;

  // Update total time.
  this->totalPlugPortTime += dt;

  // Compute average.
  this->avgPlugPortDistance =
      this->accumPlugPortDistance / this->totalPlugPortTime;

  // Store new values.
  this->lastPlugPortStamp = newTime;
  this->plugPortDistance = distance;

  return true;
}

//////////////////////////////////////////////////
double ScoringTier2::GetPlugPortDistance() const {
  return this->plugPortDistance;
}

//////////////////////////////////////////////////
double ScoringTier2::GetAvgPlugPortDistance() const {
  return this->avgPlugPortDistance;
}

//////////////////////////////////////////////////
void ScoringTier2::ResetPlugPortDistance() {
  this->plugPortDistance = 0.0;
  this->avgPlugPortDistance = 0.0;
  this->accumPlugPortDistance = 0.0;
  this->totalPlugPortTime = 0.0;
  this->lastPlugPortStamp = -1.0;
}

}  // namespace aic_scoring
