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

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "aic_scoring/ScoringTier2.hh"

namespace aic_scoring
{
//////////////////////////////////////////////////
ScoringTier2::ScoringTier2(rclcpp::Node *_node)
  : node(_node)
{
}

//////////////////////////////////////////////////
ScoringTier2Node::ScoringTier2Node()
  : Node("score_tier2_node")
{
  this->score = std::make_unique<aic_scoring::ScoringTier2>(this);
}

//////////////////////////////////////////////////
bool ScoringTier2Node::ParseStats(const std::string &_yamlFile)
{
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(_yamlFile);
  }
  catch (const YAML::BadFile &_e)
  {
    std::cerr << "Unable to open YAML file [" << _yamlFile << "]" << std::endl;
    return false;
  }

  // Sanity check: We should have a [plugs] map.
  if (!config["plugs"])
  {
    std::cerr << "Unable to find [plugs] in tier2.yaml" << std::endl;
    return false;
  }

  // Sanity check: We should have a sequence of [plug]
  auto plugs = config["plugs"];
  if (!plugs.IsSequence())
  {
    std::cerr << "Unable to find sequence of plugs within [plugs]"
              << std::endl;
    return false;
  }

  for (std::size_t i = 0u; i < plugs.size(); i++)
  {
    auto newPlug = plugs[i];

    // Sanity check: The key should be "plug".
    if (!newPlug["plug"])
    {
      std::cerr << "Unrecognized element. It should be [plug]" << std::endl;
      return false;
    }

    Pluggable plug;
    auto plugProperties = newPlug["plug"];
    if (!plugProperties.IsMap())
    {
      std::cerr << "Unable to find properties within [plug]" << std::endl;
      return false;
    }

    if (!plugProperties["name"])
    {
      std::cerr << "Unable to find [name] within [plug]" << std::endl;
      return false;
    }
    plug.name = plugProperties["name"].as<std::string>();

    if (!plugProperties["type"])
    {
      std::cerr << "Unable to find [type] within [plug]" << std::endl;
      return false;
    }

    plug.type = plugProperties["type"].as<std::string>();
    this->score->plugs.insert({plug.name, std::move(plug)});
  }

  // Sanity check: We should have a [ports] map.
  if (!config["ports"])
  {
    std::cerr << "Unable to find [ports] in tier2.yaml" << std::endl;
    return false;
  }

  // Sanity check: We should have a sequence of [port]
  auto ports = config["ports"];
  if (!ports.IsSequence())
  {
    std::cerr << "Unable to find sequence of ports within [ports]"
              << std::endl;
    return false;
  }

  for (std::size_t i = 0u; i < ports.size(); i++)
  {
    auto newPort = ports[i];

    // Sanity check: The key should be "port".
    if (!newPort["port"])
    {
      std::cerr << "Unrecognized element. It should be [port]" << std::endl;
      return false;
    }

    Pluggable port;
    auto portProperties = newPort["port"];
    if (!portProperties.IsMap())
    {
      std::cerr << "Unable to find properties within [port]" << std::endl;
      return false;
    }

    if (!portProperties["name"])
    {
      std::cerr << "Unable to find [name] within [port]" << std::endl;
      return false;
    }
    port.name = portProperties["name"].as<std::string>();

    if (!portProperties["type"])
    {
      std::cerr << "Unable to find [type] within [port]" << std::endl;
      return false;
    }

    port.type = portProperties["type"].as<std::string>();
    this->score->ports.insert({port.name, std::move(port)});
  }

  // Populate pluggableMap.
  for (const auto& [plugName, plugInfo] : this->score->plugs)
  {
    for (const auto& [portName, portInfo] : this->score->ports)
    {
      if (plugInfo.type == portInfo.type)
      {
        std::string connectionName = plugName + "&" + portName;
        this->score->pluggableMap.insert({connectionName, 0});
      }
    }
  }

  return true;
}

}  // namespace aic_scoring

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  // Sanity check: There should be one argument.
  if (argc != 2)
  {
    std::cerr << "Usage: scoring_tier1 <tier1_yaml_file>" << std::endl;
    return -1;
  }

  rclcpp::init(argc, argv);

  auto scoringTier2 = std::make_shared<aic_scoring::ScoringTier2Node>();
  std::string configFile = std::string(argv[1]);
  if (!scoringTier2->ParseStats(configFile))
    return -1;

  // Debug.
  for (const auto& [connection, distance] : scoringTier2->score->pluggableMap)
    std::cout << connection << ": " << distance << " m." << std::endl;

  rclcpp::spin(scoringTier2);
  rclcpp::shutdown();
  return 0;
}
