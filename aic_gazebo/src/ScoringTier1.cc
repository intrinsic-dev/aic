
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
#include <string>
#include <vector>

#include "aic_gazebo/ScoringTier1.hh"

namespace aic_gazebo 
{
//////////////////////////////////////////////////
TopicStatsTier1::TopicStatsTier1(std::string &_topic)
  : Node("statsTier1_" + _topic),
    lastTimestamp(std::chrono::steady_clock::now()),
    topic(_topic)
{
  this->subscription = this->create_subscription<std_msgs::msg::String>(
    _topic, 10, std::bind(&TopicStatsTier1::TopicCallback, this, 
    std::placeholders::_1));
}

//////////////////////////////////////////////////
double TopicStatsTier1::Median()
{
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
  auto delta = (now - this->lastTimestamp).count();
  this->lastTimestamp = now;

  // Insert while maintaining the sort order.
  auto it = std::lower_bound(
    this->stats.deltas.begin(), this->stats.deltas.end(), delta);
  this->stats.deltas.insert(it, delta);

  this->stats.size++;
  this->stats.median = this->Median();
}

//////////////////////////////////////////////////
void TopicStatsTier1::TopicCallback(const std_msgs::msg::String &/*_msg*/)
{
  this->Update();
}

//////////////////////////////////////////////////
ScoringTier1::ScoringTier1(std::vector<std::string> &_topics)
{
  for (auto topic : _topics)
    this->allStats.insert({topic, std::make_unique<TopicStatsTier1>(topic)});
}

}  // namespace aic_gazebo
