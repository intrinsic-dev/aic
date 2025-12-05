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
#include "aic_scoring/ScoringTier1.hh"

namespace aic_scoring
{
//////////////////////////////////////////////////
TopicStatsTier1::TopicStatsTier1(rclcpp::Node *_node,
                                 const StatsTier1 &_topicStats)
  : lastTimestamp(std::chrono::steady_clock::now()),
    stats(_topicStats),
    node(_node)
{
  if (!_node)
  {
    std::cerr << "[TopicStatsTier1]: null ROS node. Aborting." << std::endl;
    return;
  }

  this->subscription = this->node->create_generic_subscription(
      this->stats.topicName, this->stats.topicType, rclcpp::QoS(10),
      std::bind(&TopicStatsTier1::TopicCallback, this,
      std::placeholders::_1));
}

//////////////////////////////////////////////////
double TopicStatsTier1::Median() const
{
  if (this->stats.deltas.empty())
    return 0.0;

  // Check if the number of elements is odd.
  if (this->stats.size % 2 != 0)
    return this->stats.deltas[this->stats.size / 2];

  // If the number of elements is even, return the average
  // of the two middle elements.
  return (this->stats.deltas[(this->stats.size - 1) / 2] +
          this->stats.deltas[this->stats.size / 2]) / 2.0;
}

//////////////////////////////////////////////////
void TopicStatsTier1::Update()
{
  auto now = std::chrono::steady_clock::now();
  auto delta = std::chrono::duration<double>(now - this->lastTimestamp).count();

  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->lastTimestamp = now;

    // Insert while maintaining the sort order.
    auto it = std::lower_bound(
      this->stats.deltas.begin(), this->stats.deltas.end(), delta);
    this->stats.deltas.insert(it, delta);

    this->stats.size++;
    this->stats.median = this->Median();
    this->stats.passed = this->stats.size >= this->stats.minMessages &&
                         this->stats.median <= this->stats.median;

    RCLCPP_INFO_STREAM(
      this->node->get_logger(),
      "\nTopic: " << this->stats.topicName << std::endl <<
      "Type: " << this->stats.topicType << std::endl <<
      "Size: " << this->stats.size << std::endl <<
      "Median: " << this->stats.median << std::endl <<
      "Passed: " << std::boolalpha << this->stats.passed << std::noboolalpha <<
      std::endl <<
      "--" << std::endl);
  }
}

//////////////////////////////////////////////////
StatsTier1 TopicStatsTier1::Stats() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return this->stats;
}

//////////////////////////////////////////////////
void TopicStatsTier1::TopicCallback(std::shared_ptr<rclcpp::SerializedMessage>)
{
  this->Update();
}

//////////////////////////////////////////////////
ScoringTier1::ScoringTier1()
  : Node("score_tier1_node")
{
}

//////////////////////////////////////////////////
bool ScoringTier1::ParseStats(const std::string &_yamlFile)
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

  // Sanity check: We should have a [topics] map.
  if (!config["topics"])
  {
    std::cerr << "Unable to find [topics] in tier1.yaml" << std::endl;
    return false;
  }

  // Sanity check: We should have a sequence of [topic]
  auto topics = config["topics"];
  if (!topics.IsSequence())
  {
    std::cerr << "Unable to find sequence of topics within [topics]"
              << std::endl;
    return false;
  }

  for (std::size_t i = 0u; i < topics.size(); i++)
  {
    auto newTopic = topics[i];

    // Sanity check: The key should be "topic".
    if (!newTopic["topic"])
    {
      std::cerr << "Unrecognized element. It should be [topic]" << std::endl;
      return false;
    }

    StatsTier1 stats;
    auto topicProperties = newTopic["topic"];
    if (!topicProperties.IsMap())
    {
      std::cerr << "Unable to find properties within [topic]" << std::endl;
      return false;
    }

    if (!topicProperties["name"])
    {
      std::cerr << "Unable to find [name] within [topic]" << std::endl;
      return false;
    }
    stats.topicName = topicProperties["name"].as<std::string>();

    if (!topicProperties["type"])
    {
      std::cerr << "Unable to find [type] within [topic]" << std::endl;
      return false;
    }
    stats.topicType = topicProperties["type"].as<std::string>();

    if (!topicProperties["min_messages"])
    {
      std::cerr << "Unable to find [min_messages] within [topic]" << std::endl;
      return false;
    }
    stats.minMessages = topicProperties["min_messages"].as<double>();

    if (!topicProperties["max_median_time"])
    {
      std::cerr << "Unable to find [max_median_time] within [topic]"
                << std::endl;
      return false;
    }
    stats.maxMedianTime = topicProperties["max_median_time"].as<double>();

    auto topicStats = std::make_unique<TopicStatsTier1>(this, stats);
    this->allStats.insert({stats.topicName, std::move(topicStats)});
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

  auto scoringTier1 = std::make_shared<aic_scoring::ScoringTier1>();
  std::string configFile = std::string(argv[1]);
  if (!scoringTier1->ParseStats(configFile))
    return -1;

  rclcpp::spin(scoringTier1);
  rclcpp::shutdown();
  return 0;
}
