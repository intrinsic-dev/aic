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
#include <string>
#include <vector>

namespace aic_scoring {
//////////////////////////////////////////////////
bool ScoringTier2::ParsePlugsFromYaml(YAML::Node &_yaml,
                                     std::map<std::string, Pluggable> &_plugs) {
  if (!_yaml["plugs"]) {
    std::cerr << "Unable to find [plugs] in yaml" << std::endl;
    return false;
  }

  // Sanity check: We should have a sequence of [plug]
  auto plugs = _yaml["plugs"];
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
        !_plugs.insert({plug.name, std::move(plug)}).second) {
      std::cerr << "Plug [" << name << "] repeated. Ignoring." << std::endl;
    }
  }
  return true;
}

//////////////////////////////////////////////////
bool ScoringTier2::ParsePortsFromYaml(YAML::Node &_yaml,
                                     std::map<std::string, Pluggable> &_ports) {
  if (!_yaml["ports"]) {
    std::cerr << "Unable to find [ports] in yaml" << std::endl;
    return false;
  }

  // Sanity check: We should have a sequence of [port]
  auto ports = _yaml["ports"];
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
        !_ports.insert({port.name, std::move(port)}).second) {
      std::cerr << "Port [" << name << "] repeated. Ignoring." << std::endl;
    }
  }
  return true;
}

//////////////////////////////////////////////////
ScoringTier2::ScoringTier2(rclcpp::Node *_node, YAML::Node *_config)
    : node(_node) {
  this->yamlNode = YAML::Clone(*_config);

  if (!this->ParseStats()) return;

  // Debug.
  // for (const auto &[connection, distance] : this->pluggableMap)
  //   std::cout << connection << ": " << distance << " m." << std::endl;
}

//////////////////////////////////////////////////
bool ScoringTier2::ParseStats() {
  if (!ParsePlugsFromYaml(this->yamlNode, this->plugs))
    return false;

  if (!ParsePortsFromYaml(this->yamlNode, this->ports))
    return false;

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
