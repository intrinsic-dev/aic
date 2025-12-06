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
ScoringTier2::ScoringTier2()
  : Node("score_tier2_node")
{
}

//////////////////////////////////////////////////
bool ScoringTier2::ParseStats(const std::string &_yamlFile)
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

  // // Sanity check: We should have a [topics] map.
  // if (!config["topics"])
  // {
  //   std::cerr << "Unable to find [topics] in tier1.yaml" << std::endl;
  //   return false;
  // }

  // // Sanity check: We should have a sequence of [topic]
  // auto topics = config["topics"];
  // if (!topics.IsSequence())
  // {
  //   std::cerr << "Unable to find sequence of topics within [topics]"
  //             << std::endl;
  //   return false;
  // }

  // for (std::size_t i = 0u; i < topics.size(); i++)
  // {
  //   auto newTopic = topics[i];

  //   // Sanity check: The key should be "topic".
  //   if (!newTopic["topic"])
  //   {
  //     std::cerr << "Unrecognized element. It should be [topic]" << std::endl;
  //     return false;
  //   }

  //   StatsTier1 stats;
  //   auto topicProperties = newTopic["topic"];
  //   if (!topicProperties.IsMap())
  //   {
  //     std::cerr << "Unable to find properties within [topic]" << std::endl;
  //     return false;
  //   }

  //   if (!topicProperties["name"])
  //   {
  //     std::cerr << "Unable to find [name] within [topic]" << std::endl;
  //     return false;
  //   }
  //   stats.topicName = topicProperties["name"].as<std::string>();

  //   if (!topicProperties["type"])
  //   {
  //     std::cerr << "Unable to find [type] within [topic]" << std::endl;
  //     return false;
  //   }
  //   stats.topicType = topicProperties["type"].as<std::string>();

  //   if (!topicProperties["min_messages"])
  //   {
  //     std::cerr << "Unable to find [min_messages] within [topic]" << std::endl;
  //     return false;
  //   }
  //   stats.minMessages = topicProperties["min_messages"].as<double>();

  //   if (!topicProperties["max_median_time"])
  //   {
  //     std::cerr << "Unable to find [max_median_time] within [topic]"
  //               << std::endl;
  //     return false;
  //   }
  //   stats.maxMedianTime = topicProperties["max_median_time"].as<double>();

  //   auto topicStats = std::make_unique<TopicStatsTier1>(this, stats);
  //   this->allStats.insert({stats.topicName, std::move(topicStats)});
  // }
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

  auto scoringTier2 = std::make_shared<aic_scoring::ScoringTier2>();
  std::string configFile = std::string(argv[1]);
  if (!scoringTier2->ParseStats(configFile))
    return -1;

  rclcpp::spin(scoringTier2);
  rclcpp::shutdown();
  return 0;
}
