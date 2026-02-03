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

#include <aic_control_interfaces/msg/joint_motion_update.hpp>
#include <aic_control_interfaces/msg/motion_update.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/buffer_core.hpp>

#include <aic_scoring/TierScore.hh>

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
    using JointStateMsg = sensor_msgs::msg::JointState;
    using TFMsg = tf2_msgs::msg::TFMessage;
    using ContactsMsg = ros_gz_interfaces::msg::Contacts;
    using WrenchMsg = geometry_msgs::msg::WrenchStamped;
    using JointMotionUpdateMsg = aic_control_interfaces::msg::JointMotionUpdate;
    using MotionUpdateMsg = aic_control_interfaces::msg::MotionUpdate;

    enum class State {
      Idle,
      Recording,
      Scoring
    };

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

    /// \brief Topic to subscribe for pose commands sent to the controller.
    public: static constexpr const char* kMotionUpdateTopic = "/aic_controller/pose_commands";

    /// \brief Topic to subscribe for joint commands sent to the controller.
    public: static constexpr const char* kJointMotionUpdateTopic = "/aic_controller/joint_commands";

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

    /// \brief Compute the score the bag that we just recorded.
    /// \return A pair with the Tier2 and Tier3 scores.
    public: std::pair<Tier2Score, Tier3Score> ComputeScore();

    /// \brief Populate the scoring input params from a YAML file.
    /// \param[in] _config YAML configuration for the node
    private: bool ParseStats(YAML::Node _config);

    /// \brief Get the topics required that are currently not being published.
    /// \return An unordered_set with the missing required topic names.
    public: std::set<std::string> GetMissingRequiredTopics() const;

    /// \brief Set the start time for the task.
    /// \param[in] _time The start time of the task.
    public: void SetTaskStartTime(const rclcpp::Time& _time);

    /// \brief Set the end time for the task.
    /// \param[in] _time The end time of the task.
    public: void SetTaskEndTime(const rclcpp::Time& _time);

    /// \brief Callback for joint state messages received while scoring.
    /// \param[in] _msg The received message.
    private: void JointStateCallback(const JointStateMsg& _msg);

    /// \brief Callback for tf messages received while scoring.
    /// \param[in] _msg The received message.
    private: void TfCallback(const TFMsg& _msg);

    /// \brief Callback for static tf messages received while scoring.
    /// \param[in] _msg The received message.
    private: void TfStaticCallback(const TFMsg& _msg);

    /// \brief Callback for contact messages received while scoring.
    /// \param[in] _msg The received message.
    private: void ContactsCallback(const ContactsMsg& _msg);

    /// \brief Callback for force torque sensor wrenches received while scoring.
    /// \param[in] _msg The received message.
    private: void WrenchCallback(const WrenchMsg& _msg);

    /// \brief Callback for pose commands received while scoring.
    /// \param[in] _msg The received message.
    private: void MotionUpdateCallback(const MotionUpdateMsg& _msg);

    /// \brief Callback for joint commands received while scoring.
    /// \param[in] _msg The received message.
    private: void JointMotionUpdateCallback(const JointMotionUpdateMsg& _msg);

    /// \brief Calculates the distance between the plug and the port.
    /// \param[in] _timestamp Time to check the distance
    /// \return Distance between plug and port at the end of the task. nullopt if failed
    private: std::optional<double> GetPlugPortDistance(tf2::TimePoint t) const;

    /// \brief Calculates the tier 3 score based on the distance between plug and port.
    /// \return Scoring for the distance category
    private: Tier3Score GetDistanceScore() const;

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

    /// \brief The URI of the bag currently being processed.
    private: std::string bagUri;

    /// \brief The time the task started, used for computing task duration.
    // TODO(luca) Either have an API to reset all state or destroy + rebuild
    // this class between scoring sessions
    private: std::optional<rclcpp::Time> task_start_time;

    /// \brief The time the task ended, used for computing task duration.
    private: std::optional<rclcpp::Time> task_end_time;

    /// \brief State the scoring system is in.
    private: State state = State::Idle;

    /// \brief Buffer to compute tf for scoring.
    private: tf2::BufferCore tf2_buffer;

    /// \brief Timestamps of received tfs to be used for distance calculation
    private: std::set<tf2::TimePoint> timestamps;

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
