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

#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>

#include <aic_control_interfaces/msg/controller_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
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

  /// \brief Utility class to split incoming namespace strings into a struct
  struct InsertionNamespace
  {
    std::string cable_name;
    std::size_t cable_end;
    std::string module_name;
    std::string port_name;

    static std::optional<InsertionNamespace> make(std::string msg);
  };

  // The Tier2 scoring interface.
  class ScoringTier2 : public std::enable_shared_from_this<ScoringTier2>
  {
    using TFMsg = tf2_msgs::msg::TFMessage;
    using ContactsMsg = ros_gz_interfaces::msg::Contacts;
    using WrenchMsg = geometry_msgs::msg::WrenchStamped;
    using ControllerStateMsg = aic_control_interfaces::msg::ControllerState;
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

    /// \brief Topic to subscribe for tf.
    public: static constexpr const char* kScoringTfTopic = "/scoring/tf";

    /// \brief Topic to subscribe for contacts.
    public: static constexpr const char* kContactsTopic = "/aic/gazebo/contacts/off_limit";

    /// \brief Topic to subscribe for force torque sensor wrench.
    public: static constexpr const char* kWrenchTopic = "/fts_broadcaster/wrench";

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
    /// \param[in] _gripperFrame Name of the gripper frame.
    /// \param[in] _connections Connections for this task.
    public: ScoringTier2(rclcpp::Node *_node, const std::string &_gripperFrame, const CableConnections &_connections);

    /// \brief Start recording all scoring topics.
    /// \return True if the scoring system is ready.
    /// \param[in] _max_task_time The maximum time to record for, used for tf buffer size.
    /// \param[in] _use_sim_time Whether to wait for simulation TFs.
    public: bool StartRecording(const std::chrono::seconds &_max_task_time,
                                const bool _use_sim_time = true);

    /// \brief Stop recording all scoring topics.
    /// \return True if the bag was closed correctly.
    public: bool StopRecording();

    /// \brief Compute the score the bag that we just recorded.
    /// \return A pair with the Tier2 and Tier3 scores.
    public: std::pair<Tier2Score, Tier3Score> ComputeScore();

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
    /// \return Scoring for the trajectory jerk score.
    private: Tier2Score::CategoryScore GetTrajectoryJerkScore() const;

    /// \brief Calculates score for trajectory efficiency (path length).
    /// \return Scoring for the trajectory efficiency category.
    private: Tier2Score::CategoryScore GetTrajectoryEfficiencyScore() const;

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
    /// \param[in] t Time to check the distance
    /// \param[in] index The index of the connection to check
    /// \return Distance between plug and port at the end of the task. nullopt if failed
    private: std::optional<double> GetPlugPortDistance(tf2::TimePoint t, std::size_t index) const;

    /// \brief Calculates the distance between the plug and the port at the start of the task.
    /// \param[in] index The index of the connection to check
    /// \return Distance between plug and port at the end of the task. nullopt if failed
    private: std::optional<double> GetStartPlugPortDistance(std::size_t index) const;

    /// \brief Calculates the penalty (if any) for excessive insertion force.
    /// \return Scoring for the insertion force category
    private: Tier2Score::CategoryScore GetInsertionForceScore() const;

    /// \brief Calculates the tier 3 score based on the distance between plug and port.
    /// \param[in] index The index of the connection to score
    /// \return Scoring for the distance category
    private: Tier3Score GetDistanceScore(std::size_t index) const;

    /// \return Compute Tier3 score
    /// \param[in] index The index of the connection to score
    private: Tier3Score ComputeTier3Score(std::size_t index) const;

    /// \brief Compute and combine tier 3 scores.
    /// \return The tier3 score for the whole cable.
    private: Tier3Score CombineTier3Score() const;

    /// \brief Calculates the penalty (if any) for contacts with off limit entities.
    /// \return Scoring for the off limit contacts category
    private: Tier2Score::CategoryScore GetContactsScore() const;

    /// \brief Calculates the score for task duration.
    /// \return Scoring for the task duration category.
    private: Tier2Score::CategoryScore GetTaskDurationScore() const;

    /// \brief Wait for the cable and gripper TFs to be received.
    /// \return True if the transform were received, false if timeout occurred.
    private: bool WaitForTfs();

    /// \brief Pointer to a node.
    private: rclcpp::Node *node;

    /// \brief Gripper frame name.
    private: std::string gripperFrame;

    /// \brief Connection.
    private: std::optional<CableConnections> connection;

    /// \brief Subscriptions for all the scoring data.
    private: rclcpp::Subscription<TFMsg>::SharedPtr static_tf_sub;
    private: rclcpp::Subscription<TFMsg>::SharedPtr tf_sub;
    private: rclcpp::Subscription<TFMsg>::SharedPtr scoring_tf_sub;
    private: rclcpp::Subscription<ContactsMsg>::SharedPtr contacts_sub;
    private: rclcpp::Subscription<WrenchMsg>::SharedPtr wrench_sub;
    private: rclcpp::Subscription<StringMsg>::SharedPtr insertion_event_sub;
    private: rclcpp::Subscription<StringMsg>::SharedPtr cable_activated_sub;
    private: rclcpp::Subscription<ControllerStateMsg>::SharedPtr controller_state_sub;

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

    /// \brief The insertion port namespaces as detected by the cable plugins.
    /// Empty string means no insertion event detected.
    private: std::unordered_set<std::string> insertionPortNamespaces;

    /// \brief The cable that was activated and is being tracked.
    private: std::optional<std::string> trackedCable;
  };
}
#endif
