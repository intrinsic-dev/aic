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

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

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
    /// \brief Topic to subscribe for joint states.
    public: static constexpr const char* kJointStateTopic = "/joint_states";

    /// \brief Topic to subscribe for static tf.
    public: static constexpr const char* kTfStaticTopic = "/scoring/tf_static";

    /// \brief Topic to subscribe for tf.
    public: static constexpr const char* kTfTopic = "/scoring/tf";

    /// \brief Topic to subscribe for contacts.
    public: static constexpr const char* kContactsTopic = "/aic/gazebo/contacts/off_limit";

    /// \brief Topic to subscribe for force torque sensor wrench.
    public: static constexpr const char* kWrenchTopic = "/axia80_m20/wrench";

    /// \brief Class constructor.
    /// \param[in] _node Pointer to the ROS node.
    public: ScoringTier2(rclcpp::Node *_node);

    /// \brief Initialize the node's subscriptions.
    public: bool Initialize();

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

    /// \brief Get the topics required that are currently not being published.
    /// \return An unordered_set with the missing required topic names.
    public: std::set<std::string> GetMissingRequiredTopics() const;

    /// \brief Pointer to a node.
    private: rclcpp::Node *node;

    /// \brief Connections.
    private: std::vector<Connection> connections;

    /// \brief A rosbag2 writer.
    private: rosbag2_cpp::Writer bagWriter;

    /// \brief Whether the bag is open or not.
    private: bool bagOpen = false;

    /// \brief Subscription for the joint state.
    private: rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub;

    /// \brief Subscription for the tf.
    private: rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfSub;

    /// \brief Subscription for the static tf.
    private: rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfStaticSub;

    /// \brief Subscription for the gazebo off limit contacts.
    private: rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contactsSub;

    /// \brief Subscription for the force torque sensor wrench.
    private: rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrenchSub;

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
