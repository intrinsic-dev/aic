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
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <gz/math/Pose3.hh>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#ifndef AIC_SCORING__SCORING_TIER2_HH_
#define AIC_SCORING__SCORING_TIER2_HH_

namespace aic_scoring
{
  /// \brief Tier2 POD.
  class Pluggable
  {
    /// \brief Plug/port name.
    public: std::string name;

    /// \brief Plug/port type.
    public: std::string type;

    /// \brief Position.
    public: gz::math::Vector3d position;
  };

  // The Tier2 scoring interface.
  class ScoringTier2
  {
    /// \brief Class constructor.
    /// \param[in] _node Pointer to the ROS node.
    /// \param[in] _config YAML config node.
    public: ScoringTier2(rclcpp::Node *_node,
                         YAML::Node *_config);

    /// \brief Populate the scoring input params from a YAML file.
    public: bool ParseStats();

    /// \brief Store the current distance cable-connector.
    public: void Update();

    /// \brief Start recording all scoring topics.
    /// \return True if the bag was opened correctly and it's ready to record.
    public: bool StartRecording(const std::string &_filename);

    /// \brief Stop recording all scoring topics.
    /// \return True if the bag was closed correctly.
    public: bool StopRecording();

    /// \brief All pluggable plugs.
    public: std::map<std::string, Pluggable> plugs;

    /// \brief All pluggable ports.
    public: std::map<std::string, Pluggable> ports;

    /// \brief Plug<->port connections.
    /// The first key is always the plug, followed by "&", followed by port.
    /// The value is the distance (meters) between the plug and the port.
    public: std::map<std::string, double> pluggableMap;

    /// \brief Pointer to a node.
    private: rclcpp::Node *node;

    /// \brief Subscription.
    private: rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub1;

    /// \brief Subscription.
    private: rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub2;

    /// \brief Subscription.
    private: rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub3;

    /// \brief Subscription.
    private: rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr sub4;

    /// \brief A YAML node.
    private: YAML::Node yamlNode;

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
