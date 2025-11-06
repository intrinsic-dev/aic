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
TopicStatsTier1::TopicStatsTier1(rclcpp::Node *_node, std::string &_topicName,
  std::string &_topicType)
  : lastTimestamp(std::chrono::steady_clock::now()),
    topicName(_topicName),
    topicType(_topicType),
    node(_node)
{
  if (!_node)
  {
    std::cerr << "[TopicStatsTier1]: null ROS node. Aborting." << std::endl;
    return;
  }

  this->subscription = this->node->create_generic_subscription(
      this->topicName, this->topicType, rclcpp::QoS(10),
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

    RCLCPP_INFO_STREAM(
      this->node->get_logger(),
      "\nTopic: " << this->topicName << std::endl <<
      "Type: " << this->topicType << std::endl <<
      "Size: " << this->stats.size << std::endl <<
      "Median: " << this->stats.median << std::endl <<
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
ScoringTier1::ScoringTier1(
  std::vector<std::pair<std::string, std::string>> &_topicsAndTypes)
  : Node("score_tier1_node")
{
  for (auto& [topicName, topicType] : _topicsAndTypes)
  {
    auto topicStats = std::make_unique<TopicStatsTier1>(
      this, topicName, topicType);
    this->allStats.insert({topicName, std::move(topicStats)});
  }
}

}  // namespace aic_scoring

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  std::vector<std::pair<std::string, std::string>> topicsAndTypes =
    {
      {"/foo", "std_msgs/msg/String"},
      {"/bar", "std_msgs/msg/String"}
    };
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aic_scoring::ScoringTier1>(topicsAndTypes));
  rclcpp::shutdown();
  return 0;
}
