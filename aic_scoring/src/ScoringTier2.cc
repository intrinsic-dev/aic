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

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <string>
#include <vector>

namespace aic_scoring {
//////////////////////////////////////////////////
ScoringTier2::ScoringTier2(rclcpp::Node *_node, YAML::Node *_config)
    : node(_node) {
  this->yamlNode = YAML::Clone(*_config);

  if (!this->ParseStats()) return;

  // Debug.
  for (const auto &[connection, distance] : this->pluggableMap)
    std::cout << connection << ": " << distance << " m." << std::endl;

  // Subscribe to all topics relevant for scoring.
  this->sub1 = this->node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    10,
    [this](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
      // Bag the data.
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->bagOpen) {
        rclcpp::Time time_stamp = this->node->now();
        this->bagWriter.write(
          msg, "/joint_states", "sensor_msgs/msg/JointState", time_stamp);
      }
    }
  );

  this->sub2 = this->node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/scoring/tf",
    10,
    [this](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
      // Bag the data.
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->bagOpen) {
        rclcpp::Time time_stamp = this->node->now();
        this->bagWriter.write(
          msg, "/scoring/tf", "tf2_msgs::msg::TFMessage", time_stamp);
      }
    }
  );

  this->sub3 = this->node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/scoring/tf_static",
    10,
    [this](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
      // Bag the data.
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->bagOpen) {
        rclcpp::Time time_stamp = this->node->now();
        this->bagWriter.write(
          msg, "/scoring/tf_static", "tf2_msgs::msg::TFMessage", time_stamp);
      }
    }
  );

  this->sub4 = this->node->create_subscription<ros_gz_interfaces::msg::Contacts>(
    "/aic/gazebo/contacts/off_limit",
    10,
    [this](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
      // Bag the data.
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->bagOpen) {
        rclcpp::Time time_stamp = this->node->now();
        this->bagWriter.write(
          msg, "/aic/gazebo/contacts/off_limit",
          "ros_gz_interfaces::msg::Contacts", time_stamp);
      }  
    }
  );
}

//////////////////////////////////////////////////
bool ScoringTier2::StartRecording(const std::string &_filename) {
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->bagOpen) {
    RCLCPP_ERROR(this->node->get_logger(), "Bag already opened.");
    return false;
  }

  this->bagWriter.open(_filename);
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
    std::cout << "Name: " << plug.name << std::endl;

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

  return true;
}

//////////////////////////////////////////////////
// void ScoringTier2::JointStatesCallback(
//         const sensor_msgs::msg::JointState::SharedPtr msg) const {

// }

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

}  // namespace aic_scoring
