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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>

#include <aic_control_interfaces/msg/controller_state.hpp>
#include <aic_control_interfaces/msg/joint_motion_update.hpp>
#include <aic_control_interfaces/msg/motion_update.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
    public: std::string targetModuleName;

    /// \brief Port name.
    public: std::string portName;

    /// \brief Get the name of the plug TF
    /// \return Name of the plug TF
    public: std::string PlugTfName(const std::string& _cableName) const {
      return _cableName + "/" + plugName + "_link";
    }

    /// \brief Get the name of the port TF
    /// \return Name of the port TF
    public: std::string PortTfName() const {
      return targetModuleName + "/" + portName + "_link";
    }

    /// \brief Get the name of the port's entrance TF, used for partial insertion.
    /// \return Name of the port's entrance TF
    public: std::string PortEntranceTfName() const {
      return targetModuleName + "/" + portName + "_link_entrance";
    }
  };

  /// \brief Connections for both end of the cable
  struct CableConnections
  {
    public: std::array<Connection, 2> data;

    CableConnections(const Connection& conn0, const Connection& conn1) :
      data({conn0, conn1}) {}
  };

  /// \brief Topic info POD.
  struct TopicInfo
  {
    /// \brief Topic name.
    std::string name;

    /// \brief Topic type (e.g., sensor_msgs/msg/JointState).
    std::string type;

    /// \brief Whether topic is latched or not
    bool latched{false};
  };

  // The Tier2 scoring interface.
  class ScoringTier2
  {
    using JointStateMsg = sensor_msgs::msg::JointState;
    using TFMsg = tf2_msgs::msg::TFMessage;
    using ContactsMsg = ros_gz_interfaces::msg::Contacts;
    using WrenchMsg = geometry_msgs::msg::WrenchStamped;
    using ControllerStateMsg = aic_control_interfaces::msg::ControllerState;
    using JointMotionUpdateMsg = aic_control_interfaces::msg::JointMotionUpdate;
    using MotionUpdateMsg = aic_control_interfaces::msg::MotionUpdate;
    using StringMsg = std_msgs::msg::String;
    using TransformStampedMsg = geometry_msgs::msg::TransformStamped;
    using Vector3Msg = geometry_msgs::msg::Vector3;

    enum class State {
      Idle,
      Recording,
      Scoring
    };

    /// \brief Topic to subscribe for static tf.
    public: static constexpr const char* kTfStaticTopic = "/scoring/tf_static";

    /// \brief Topic to subscribe for tf.
    public: static constexpr const char* kTfTopic = "/tf";

    /// \brief Topic to subscribe for cable_0 tf.
    public: static constexpr const char* kCable0TfTopic = "/cable_0/tf";

    /// \brief Topic to subscribe for cable_1 tf.
    public: static constexpr const char* kCable1TfTopic = "/cable_1/tf";

    /// \brief Topic to subscribe for cable_2 tf.
    public: static constexpr const char* kCable2TfTopic = "/cable_2/tf";

    /// \brief Topic to subscribe for cable_3 tf.
    public: static constexpr const char* kCable3TfTopic = "/cable_3/tf";

    /// \brief Topic to subscribe for cable_4 tf.
    public: static constexpr const char* kCable4TfTopic = "/cable_4/tf";

    /// \brief Topic to subscribe for contacts.
    public: static constexpr const char* kContactsTopic = "/aic/gazebo/contacts/off_limit";

    /// \brief Topic to subscribe for force torque sensor wrench.
    public: static constexpr const char* kWrenchTopic = "/fts_broadcaster/wrench";

    /// \brief Topic to subscribe for pose commands sent to the controller.
    public: static constexpr const char* kMotionUpdateTopic = "/aic_controller/pose_commands";

    /// \brief Topic to subscribe for insertion event
    public: static constexpr const char* kInsertionEventTopic =
        "/scoring/insertion_event";

    /// \brief Topic to subscribe for cable activation event
    public: static constexpr const char* kCableActivatedTopic =
        "/scoring/cable_activated";

    /// \brief Topic to subscribe for controller state used for FT sensor taring.
    public: static constexpr const char* kControllerStateTopic =
        "/aic_controller/controller_state";

    /// \brief Class constructor.
    /// \param[in] _node Pointer to the ROS node.
    public: ScoringTier2(rclcpp::Node *_node);

    /// \brief Set the gripper frame name.
    /// \param[in] _gripperFrame Gripper frame name.
    public: void SetGripperFrame(const std::string &_gripperFrame);

    /// \brief Start recording all scoring topics.
    /// \return True if the bag was opened correctly and it's ready to record.
    /// \param[in] _filename The path to the bag.
    /// \param[in] _connection Connection to monitor.
    /// \param[in] _max_task_time The maximum time to record for, used for tf buffer size.
    public: bool StartRecording(const std::string &_filename,
                const CableConnections &_connections,
                const std::chrono::seconds &_max_task_time);

    /// \brief Stop recording all scoring topics.
    /// \return True if the bag was closed correctly.
    public: bool StopRecording();

    /// \brief Compute the score the bag that we just recorded.
    /// \return A pair with the Tier2 and Tier3 scores.
    public: std::pair<Tier2Score, Tier3Score> ComputeScore();

    /// \brief Resets the internal data structures for a new scoring session
    /// \param[in] _buffer_size The tf buffer size.
    public: void Reset(const std::chrono::seconds &_buffer_size);

    /// \brief Get the topics required that are currently not being published.
    /// \return An unordered_set with the missing required topic names.
    public: std::set<std::string> GetMissingRequiredTopics() const;

    /// \brief Set the start time for the task.
    /// \param[in] _time The start time of the task.
    public: void SetTaskStartTime(const rclcpp::Time& _time);

    /// \brief Set the end time for the task.
    /// \param[in] _time The end time of the task.
    public: void SetTaskEndTime(const rclcpp::Time& _time);

    /// \brief Return whether the system is currently recording.
    /// \return True if recording, false otherwise.
    public: bool IsRecording() const;

    /// \brief Populate the scoring input params from a YAML file.
    /// \param[in] _config YAML configuration for the node
    private: bool ParseStats(YAML::Node _config);

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

    /// \brief Compute the end effector position.
    /// \param[in] t Time to check the pose.
    /// \return End effector pose at time t. nullopt if failed
    private: std::optional<TransformStampedMsg> EndEffectorPose(tf2::TimePoint t) const;

    /// \brief Callback for insertion event while scoring.
    /// \param[in] _msg The received message.
    private: void InsertionEventCallback(const StringMsg& _msg);

    /// \brief Callback for cable activation event while scoring.
    /// \param[in] _msg The received message.
    private: void CableActivatedCallback(const StringMsg& _msg);

    /// \brief Callback for controller state while scoring.
    /// \param[in] _msg The received message.
    private: void ControllerStateCallback(const ControllerStateMsg& _msg);

    /// \brief Calculates score related with the gripper trajectory jerk.
    /// \param[in] _tier3 The result of tier3 scoring.
    /// \return Scoring for the trajectory jerk score.
    private: Tier2Score::CategoryScore GetTrajectoryJerkScore(
       const Tier3Score &_tier3) const;

    /// \brief Calculates score for trajectory efficiency (path length).
    /// \param[in] _minPathLength Minimum path length for max score (meters).
    /// This is typically the initial plug-port distance.
    /// \param[in] _tier3 The result of tier3 scoring.
    /// \return Scoring for the trajectory efficiency category.
    private: Tier2Score::CategoryScore GetTrajectoryEfficiencyScore(
        double _minPathLength,
        const Tier3Score &_tier3) const;

    /// \brief Gets the transform for the specified entity at the requested time.
    /// \param[in] _t the time point to get the transform.
    /// \param[in] _target_frame the frame to get the transform for.
    /// \param[in] _reference_frame the frame that we should get the transform relative to.
    /// \param[in] _suppress_error Whether to suppress errors for failed TF lookup.
    /// \return The transform between the two frames, if available.
    private: std::optional<TransformStampedMsg> GetTransform(
                 tf2::TimePoint _t,
                 const std::string& _target_frame,
                 const std::string& _reference_frame = "default",
                 const bool _suppress_error = false) const;

    /// \brief Calculates the distance between the plug and the port.
    /// \param[in] _timestamp Time to check the distance
    /// \return Distance between plug and port at the end of the task. nullopt if failed
    private: std::optional<double> GetPlugPortDistance(tf2::TimePoint t) const;

    /// \brief Calculates the penalty (if any) for excessive insertion force.
    /// \return Scoring for the insertion force category
    private: Tier2Score::CategoryScore GetInsertionForceScore() const;

    /// \brief Calculates the tier 3 score based on the distance between plug and port.
    /// \return Scoring for the distance category
    private: Tier3Score GetDistanceScore() const;

    /// \return Compute Tier3 score
    private: Tier3Score ComputeTier3Score() const;

    /// \brief Calculates the penalty (if any) for contacts with off limit entities.
    /// \return Scoring for the off limit contacts category
    private: Tier2Score::CategoryScore GetContactsScore() const;

    /// \brief Calculates the score for task duration.
    /// \param[in] _tier3 The score for the tier3 category, to check if task was successful.
    /// \return Scoring for the task duration category.
    private: Tier2Score::CategoryScore GetTaskDurationScore(const Tier3Score& _tier3) const;

    /// \brief Wait for the cable and gripper TFs to be received.
    /// \return True if the transform were received, false if timeout occurred.
    private: bool WaitForTfs();

    /// \brief Pointer to a node.
    private: rclcpp::Node *node;

    /// \brief Topics to subscribe to.
    private: std::vector<TopicInfo> topics;

    /// \brief Connection.
    private: std::optional<CableConnections> connection;

    /// \brief Generic subscriptions for all topics.
    private: std::vector<std::shared_ptr<rclcpp::GenericSubscription>>
      subscriptions;

    /// \brief A rosbag2 writer.
    private: rosbag2_cpp::Writer bagWriter;

    /// \brief The URI of the bag currently being processed.
    private: std::string bagUri;

    /// \brief The time the task started, used for computing task duration.
    private: std::optional<rclcpp::Time> task_start_time;

    /// \brief The time the task ended, used for computing task duration.
    private: std::optional<rclcpp::Time> task_end_time;

    /// \brief State the scoring system is in.
    private: State state = State::Idle;

    /// \brief Buffer to compute tf for scoring.
    private: std::unique_ptr<tf2::BufferCore> tf2_buffer;

    /// \brief Readings from the force torque sensor, pair is timestamp
    /// and force
    private: std::vector<std::pair<double, Vector3Msg>> wrenches;

    /// \brief End effector pose tf messages.
    private: std::vector<TransformStampedMsg> endEffectorPoses;

    /// \brief End effector velocities, pair is time and linear twist
    private: std::vector<std::pair<double, Vector3Msg>> endEffectorVelocities;

    /// \brief Non empty contact messages received from the simulator.
    private: std::vector<ContactsMsg> contacts;

    /// \brief Mutex to protect the access to the bag.
    private: std::mutex mutex;

    /// \brief Gripper frame name.
    private: std::string gripperFrame;

    /// \brief The insertion port namespace as detected by the cable plugins.
    /// Empty string means no insertion event detected.
    private: std::string insertionPortNamespace;

    /// \brief The cable that was activated and is being tracked.
    private: std::optional<std::string> trackedCable;

    /// \brief Whether the tf from cable_0 was recorded.
    private: std::atomic<bool> cable0TfReceived = false;

    /// \brief Whether the tf from cable_0 was recorded.
    private: std::atomic<bool> cable1TfReceived = false;

    /// \brief Whether the tf from cable_0 was recorded.
    private: std::atomic<bool> cable2TfReceived = false;

    /// \brief Whether the tf from cable_0 was recorded.
    private: std::atomic<bool> cable3TfReceived = false;

    /// \brief Whether the tf from cable_0 was recorded.
    private: std::atomic<bool> cable4TfReceived = false;

    /// \brief Whether the tf from a gripper was recorded.
    private: std::atomic<bool> gripperTfReceived = false;

    /// \brief Whether the tf from a gripper was recorded.
    private: std::atomic<bool> staticTfReceived = false;
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
