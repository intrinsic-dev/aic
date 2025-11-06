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
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#ifndef AIC_SCORING__SCORING_TIER1_HH_
#define AIC_SCORING__SCORING_TIER1_HH_

namespace aic_scoring
{
  /// \brief Tier1 POD.
  class StatsTier1
  {
    /// \brief Topic name.
    public: std::string topicName;

    /// \brief Topic type. It should use slashes. E.g.: std_msgs/msg/String
    public: std::string topicType;

    /// \brief Min number of messages to pass.
    public: double minMessages;

    /// \brief Max median time between deltas (seconds) to pass.
    public: double maxMedianTime;

    /// \brief History of deltas in seconds (sorted).
    public: std::vector<double> deltas;

    /// \brief Number of deltas.
    public: uint64_t size = 0;

    /// \brief Delta median timestamp. This is the median elapsed time
    /// between timestamps in seconds.
    public: double median = 0.0;

    /// \brief Whether this topic stats pass or fail.
    public: bool passed = false;
  };

  // The Tier1Stats class.
  class TopicStatsTier1
  {
    /// \brief Class constructor.
    /// \param[in] _node Pointer to the ROS node.
    /// \param[in] _topicStats Topic info to track.
    public: TopicStatsTier1(rclcpp::Node *_node,
                            const StatsTier1 &_topicStats);

    /// \brief Get the current stats.
    /// \return Current stats.
    public: StatsTier1 Stats() const;

    /// \brief Topic callback;
    /// \param[in] _msg Input message (unused).
    private: void TopicCallback(
      std::shared_ptr<rclcpp::SerializedMessage> _msg);

    /// \brief Update the stats with a new timestamp.
    private: void Update();

    /// \brief Function for calculating the delta median.
    /// \return the Median value of all deltas.
    private: double Median() const;

    /// \brief Last timestamp received.
    private: std::chrono::time_point<std::chrono::steady_clock> lastTimestamp;

    /// \brief Topic stats.
    private: StatsTier1 stats;

    /// \brief ROS subscription.
    private: std::shared_ptr<rclcpp::GenericSubscription> subscription;

    /// \brief Mutex to protect stats.
    private: mutable std::mutex mutex;

    /// \brief Pointer to a node.
    private: rclcpp::Node *node;
  };

  /// \brief All topic statistics.
  /// Key: Topic name.
  /// Value: Pointer to a TopicStatsTier1 with all its associated stats.
  using AllStats =
    std::unordered_map<std::string, std::unique_ptr<TopicStatsTier1>>;

  // The Tier1 scoring class.
  class ScoringTier1 : public rclcpp::Node
  {
    /// \brief Class constructor.
    public: ScoringTier1();

    /// \brief Populate the scoring input params from a YAML file.
    /// \param[in] _yamlFile Input YAML file.
    public: bool ParseStats(const std::string &_yamlFile);

    /// \brief List of topics to track.
    public: AllStats allStats;
  };
}
#endif
