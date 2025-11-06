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
    /// \brief History of deltas in seconds (sorted).
    public: std::vector<double> deltas;

    /// \brief Number of deltas.
    public: uint64_t size = 0;

    /// \brief Delta median timestamp. This is the median elapsed time
    /// between timestamps in seconds.
    public: double median = 0.0;
  } ;

  // The Tier1Stats class.
  class TopicStatsTier1
  {
    /// \brief Class constructor.
    /// \param[in] _node Pointer to the ROS node.
    /// \param[in] _topicName Topic to track.
    /// \param[in] _topicType The topic type.
    public: TopicStatsTier1(rclcpp::Node *_node,
                            std::string &_topicName,
                            std::string &_topicType);

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

    /// \brief Topic name associated to the stats.
    private: std::string topicName;

    /// \brief Topic type associated to the stats.
    private: std::string topicType;

    /// \brief Topic stats.
    private: StatsTier1 stats;

    /// \brief ROS subscription.
    private: std::shared_ptr<rclcpp::GenericSubscription> subscription;

    /// \brief Mutex to protect stats.
    private: mutable std::mutex mutex;

    /// \brief Pointer to a node.
    private: rclcpp::Node *node;
  };

  // The Tier1 scoring class.
  class ScoringTier1 : public rclcpp::Node
  {
    /// \brief Class constructor.
    /// \param[in] _topicsAndTypes Vector of pairs.
    /// Each pair is a ROS <topic_name, topic_type> tuple.
    /// topic_type should use slashes . E.g.: std_msgs/msg/String
    public: ScoringTier1(
      std::vector<std::pair<std::string, std::string>> &_topicsAndTypes);

    /// \brief List of topics to track.
    public: std::unordered_map<std::string, std::unique_ptr<TopicStatsTier1>>
      allStats;
  };
}
#endif
