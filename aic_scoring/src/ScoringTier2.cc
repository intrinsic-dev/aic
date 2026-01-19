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
#include <rosbag2_cpp/writer.hpp>
#include <string>
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

}  // namespace aic_scoring
