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

#ifndef AIC_SCORING__SCORING_TIER2_HH_
#define AIC_SCORING__SCORING_TIER2_HH_

#include <yaml-cpp/yaml.h>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

namespace aic_scoring
{
  /// \brief Connection POD.
  struct Connection
  {
    /// \brief Plug name.
    public: std::string plugName;

    /// \brief Port name.
    public: std::string portName;

    /// \brief Plug/port type.
    public: std::string type;

    /// \brief Distance.
    public: double distance = -1;
  };

  /// \brief Topic info POD.
  struct TopicInfo
  {
    /// \brief Topic name.
    std::string name;

    /// \brief Topic type (e.g., sensor_msgs/msg/JointState).
    std::string type;
  };

  // The Tier2 scoring interface.
  class ScoringTier2
  {
    /// \brief Class constructor.
    /// \param[in] _node Pointer to the ROS node.
    public: ScoringTier2(rclcpp::Node *_node);

    /// \brief Populate the scoring input params from a YAML file.
    /// \param[in] _config YAML configuration for the node
    public: bool Initialize(YAML::Node _config);

    /// \brief Reset connections.
    /// \param[in] _connections New connections.
    public: void ResetConnections(const std::vector<Connection> &_connections);

    /// \brief Start recording all scoring topics.
    /// \return True if the bag was opened correctly and it's ready to record.
    /// \param[in] _filename The path to the bag.
    public: bool StartRecording(const std::string &_filename);

    /// \brief Stop recording all scoring topics.
    /// \return True if the bag was closed correctly.
    public: bool StopRecording();

    /// \brief Populate the scoring input params from a YAML file.
    /// \param[in] _config YAML configuration for the node
    private: bool ParseStats(YAML::Node _config);

    /// \brief Pointer to a node.
    private: rclcpp::Node *node;

    /// \brief Topics to subscribe to.
    private: std::vector<TopicInfo> topics;

    /// \brief Connections.
    private: std::vector<Connection> connections;

    /// \brief Generic subscriptions for all topics.
    private: std::vector<std::shared_ptr<rclcpp::GenericSubscription>>
      subscriptions;

    /// \brief A rosbag2 writer.
    private: rosbag2_cpp::Writer bagWriter;

    /// \brief Whether the bag is open or not.
    private: bool bagOpen = false;

    /// \brief Mutex to protect the access to the bag.
    private: std::mutex mutex;
  };

  // The Tier2 class as a node.
  class ScoringTier2Node : public rclcpp::Node
  {
    /// \brief Class constructor.
    /// \param[in] _yamlFile Path to a YAML config file.
    public: ScoringTier2Node(const std::string &_yamlFile);

    /// \brief The scoring.
    public: std::unique_ptr<ScoringTier2> score;
  };
}
#endif
