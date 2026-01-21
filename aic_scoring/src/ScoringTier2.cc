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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
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
ScoringTier2::ScoringTier2(rclcpp::Node *_node, YAML::Node *_config)
    : node(_node) {
  if (!_node) {
    std::cerr << "[ScoringTier2]: null ROS node. Aborting." << std::endl;
    return;
  }

  this->yamlNode = YAML::Clone(*_config);

  if (!this->ParseStats()) return;

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
bool ScoringTier2::ParseStats() {
  // Sanity check: We should have a [plugs] map.
  if (!this->yamlNode["plugs"]) {
    std::cerr << "Unable to find [plugs] in tier2.yaml" << std::endl;
    return false;
  }

  // Sanity check: We should have a sequence of [plug]
  auto plugs = this->yamlNode["plugs"];
  if (!plugs.IsSequence()) {
    std::cerr << "Unable to find sequence of plugs within [plugs]" << std::endl;
    return false;
  }

  for (std::size_t i = 0u; i < plugs.size(); i++) {
    auto newPlug = plugs[i];

    // Sanity check: The key should be "plug".
    if (!newPlug["plug"]) {
      std::cerr << "Unrecognized element. It should be [plug]" << std::endl;
      return false;
    }

    Pluggable plug;
    auto plugProperties = newPlug["plug"];
    if (!plugProperties.IsMap()) {
      std::cerr << "Unable to find properties within [plug]" << std::endl;
      return false;
    }

    if (!plugProperties["name"]) {
      std::cerr << "Unable to find [name] within [plug]" << std::endl;
      return false;
    }
    plug.name = plugProperties["name"].as<std::string>();

    if (!plugProperties["type"]) {
      std::cerr << "Unable to find [type] within [plug]" << std::endl;
      return false;
    }

    plug.type = plugProperties["type"].as<std::string>();

    if (auto name = plug.name;
        !this->plugs.insert({plug.name, std::move(plug)}).second) {
      std::cerr << "Plug [" << name << "] repeated. Ignoring." << std::endl;
    }
  }

  // Sanity check: We should have a [ports] map.
  if (!this->yamlNode["ports"]) {
    std::cerr << "Unable to find [ports] in tier2.yaml" << std::endl;
    return false;
  }

  // Sanity check: We should have a sequence of [port]
  auto ports = this->yamlNode["ports"];
  if (!ports.IsSequence()) {
    std::cerr << "Unable to find sequence of ports within [ports]" << std::endl;
    return false;
  }

  for (std::size_t i = 0u; i < ports.size(); i++) {
    auto newPort = ports[i];

    // Sanity check: The key should be "port".
    if (!newPort["port"]) {
      std::cerr << "Unrecognized element. It should be [port]" << std::endl;
      return false;
    }

    Pluggable port;
    auto portProperties = newPort["port"];
    if (!portProperties.IsMap()) {
      std::cerr << "Unable to find properties within [port]" << std::endl;
      return false;
    }

    if (!portProperties["name"]) {
      std::cerr << "Unable to find [name] within [port]" << std::endl;
      return false;
    }
    port.name = portProperties["name"].as<std::string>();

    if (!portProperties["type"]) {
      std::cerr << "Unable to find [type] within [port]" << std::endl;
      return false;
    }

    port.type = portProperties["type"].as<std::string>();

    if (auto name = port.name;
        !this->ports.insert({port.name, std::move(port)}).second) {
      std::cerr << "Port [" << name << "] repeated. Ignoring." << std::endl;
    }
  }

  // Populate pluggableMap.
  for (const auto &[plugName, plugInfo] : this->plugs) {
    for (const auto &[portName, portInfo] : this->ports) {
      if (plugInfo.type == portInfo.type) {
        std::string connectionName = plugName + "&" + portName;
        this->pluggableMap.insert({connectionName, 0});
      }
    }
  }

  // Parse topics to subscribe to.
  if (!this->yamlNode["topics"]) {
    std::cerr << "Unable to find [topics] in yaml file" << std::endl;
    return false;
  }

  const auto &topics = this->yamlNode["topics"];
  if (!topics.IsSequence()) {
    std::cerr << "Unable to find sequence of topics within [topics]"
              << std::endl;
    return false;
  }

  for (const auto &newTopic : topics) {
    if (!newTopic["topic"]) {
      std::cerr << "Unrecognized element. It should be [topic]" << std::endl;
      return false;
    }

    const auto &topicProperties = newTopic["topic"];
    if (!topicProperties.IsMap()) {
      std::cerr << "Unable to find properties within [topic]" << std::endl;
      return false;
    }

    TopicInfo topicInfo;

    if (!topicProperties["name"]) {
      std::cerr << "Unable to find [name] within [topic]" << std::endl;
      return false;
    }
    topicInfo.name = topicProperties["name"].as<std::string>();

    if (!topicProperties["type"]) {
      std::cerr << "Unable to find [type] within [topic]" << std::endl;
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
    this->score = std::make_unique<aic_scoring::ScoringTier2>(this, &config);
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
void ScoringTier2::ResetJerk() {
  this->poseHistory.clear();
  this->linearJerk = geometry_msgs::msg::Vector3();
  this->angularJerk = geometry_msgs::msg::Vector3();
}

}  // namespace aic_scoring
