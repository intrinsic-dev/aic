
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

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#ifndef AIC_GAZEBO__SCORING_TIER1_HH_
#define AIC_GAZEBO__SCORING_TIER1_HH_

namespace aic_gazebo
{
  class StatsTier1
  {
    /// \brief History of deltas (sorted).
    public: std::vector<double> deltas;

    /// \brief Number of deltas.
    public: uint64_t size = 0;

    /// \brief Delta median timestamp. This is the median elapsed time
    /// between timestamps.
    public: double median = 0.0;
  } ;

  // The Tier1Stats.
  class TopicStatsTier1 : public rclcpp::Node
  {
    /// \brief Class constructor.
    public: TopicStatsTier1(std::string &_topic);

    /// \brief Update the stats with a new timestamp.
    public: void Update();

    /// \brief Function for calculating the delta median.
    public: double Median();

    /// \brief Topic callback;
    private: void TopicCallback(const std_msgs::msg::String &_msg);

    /// \brief Last timestamp received.
    public: std::chrono::time_point<std::chrono::steady_clock> lastTimestamp;

    /// \brief topic associated to the stats.
    public: std::string topic;

    /// \brief Topic stats.
    public: StatsTier1 stats;

    /// \brief ROS subscription.
    private: rclcpp::Subscription<
      std_msgs::msg::String>::SharedPtr subscription;
  };

  // The Tier1 scoring.
  class ScoringTier1
  {
    /// \brief Class constructor.
    public: ScoringTier1(std::vector<std::string> &_topics);

    /// \brief List of topics to track.
    public: std::unordered_map<std::string, std::unique_ptr<TopicStatsTier1>>
      allStats;
  };
}
#endif
