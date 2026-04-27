/*
 * Copyright (C) 2026 Intrinsic Innovation LLC
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

#ifndef BRIDGES_ROBOT_CONTROL_BRIDGE_HPP_
#define BRIDGES_ROBOT_CONTROL_BRIDGE_HPP_

#include <memory>
#include <mutex>

#include "flowstate_ros_bridge/bridge_interface.hpp"
#include "intrinsic/icon/actions/tare_force_torque_sensor_info.h"
#include "intrinsic/icon/cc_client/client.h"
#include "intrinsic/icon/cc_client/session.h"
#include "intrinsic/icon/cc_client/stream.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "proto/agent_bridge.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_control_bridge/agent_bridge_info.h"
#include "robot_control_bridge/agent_bridge_joint_info.h"

// Interfaces
#include "aic_control_interfaces/msg/controller_state.hpp"
#include "aic_control_interfaces/msg/joint_motion_update.hpp"
#include "aic_control_interfaces/msg/motion_update.hpp"
#include "aic_control_interfaces/msg/target_mode.hpp"
#include "aic_control_interfaces/msg/trajectory_generation_mode.hpp"
#include "aic_control_interfaces/srv/change_target_mode.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace flowstate_ros_bridge {

using ::intrinsic::icon::Action;
using ::intrinsic::icon::ActionDescriptor;
using ::intrinsic::icon::ActionInstanceId;
using ::intrinsic::icon::Client;
using ::intrinsic::icon::OperationalStatus;
using ::intrinsic::icon::Session;

//==============================================================================
constexpr intrinsic::icon::ActionInstanceId kAgentBridgeId(1);
constexpr intrinsic::icon::ActionInstanceId kAgentBridgeJointId(2);
constexpr intrinsic::icon::ActionInstanceId kTareForceTorqueSensorId(3);

///=============================================================================
class RobotControlBridge : public BridgeInterface {
 public:
  /// Documentation inherited.
  void declare_ros_parameters(ROSNodeInterfaces ros_node_interfaces) final;

  /// Documentation inherited.
  bool initialize(ROSNodeInterfaces ros_node_interfaces,
                  std::shared_ptr<Executive> executive_client,
                  std::shared_ptr<World> world_client) final;

 private:
  void MotionUpdateCallback(
      const aic_control_interfaces::msg::MotionUpdate::SharedPtr msg);

  void JointMotionUpdateCallback(
      const aic_control_interfaces::msg::JointMotionUpdate::SharedPtr msg);

  void ChangeTargetModeCallback(
      const std::shared_ptr<
          aic_control_interfaces::srv::ChangeTargetMode::Request>
          request,
      std::shared_ptr<aic_control_interfaces::srv::ChangeTargetMode::Response>
          response);

  void RestartBridgeCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void TareForceTorqueSensorCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Starts a session on the controller server
   *
   * @return true Successfully started an ICON session
   * @return false Failed to start an ICON session
   */
  bool startControllerSession();

  /**
   * @brief Adds both the AgentBridge and AgentBridgeJoint action on the
   * controller session. Then proceeds to start the AgentBridge action on the
   * session.
   *
   * @return true Successfully added actions and started the AgentBridge actions
   * @return false Failed to add actions and and start the AgentBridge action
   */
  bool startControllerAction();

  /**
   * @brief Reset the MotionUpdate and JointMotionUpdate commands by populating
   * them with default values from the task_settings config
   *
   * @return true Successfully reset the motion commands
   * @return false Failed to reset the motion commands
   */
  bool resetMotionUpdate();

  /**
   * @brief Restarts the controller bridge. Creates a timer to retry connecting
   * to the controller server to avoid blocking other callbacks.
   *
   * @return true Successfully restarted the controller bridge
   * @return false Failed to restart the controller bridge
   */
  bool restartControllerBridge();

  absl::StatusOr<std::shared_ptr<intrinsic::Subscription>>
  CreateActionOutputStreamSubscription(
      intrinsic::SubscriptionOkCallback<intrinsic_proto::data_logger::LogItem>
          callback,
      const std::string& instance,
      const intrinsic::icon::ActionInstanceId action_instance_id);

  absl::StatusOr<std::shared_ptr<intrinsic::Subscription>>
  CreateRobotStateSubscription(
      intrinsic::SubscriptionOkCallback<intrinsic_proto::data_logger::LogItem>
          callback,
      const std::string& robot_controller_instance);

  /**
   * @brief Subscriber callback for retrieving reference TCP and reference joint
   * states
   *
   * @param log_item
   */
  void agentBridgeOutputStreamCallback(
      const intrinsic_proto::data_logger::LogItem& log_item);

  /**
   * @brief Subscriber callback for retrieving sensed TCP poses and velocity
   *
   * @param log_item
   */
  void RobotStateCallback(
      const intrinsic_proto::data_logger::LogItem& log_item);

  struct Data : public std::enable_shared_from_this<Data> {
    ROSNodeInterfaces node_interfaces_;

    // Controller Server
    std::unique_ptr<Session> session_;
    intrinsic::icon::AgentBridgeInfo::FixedParams agent_bridge_fixed_params_;
    intrinsic::icon::AgentBridgeJointInfo::FixedParams
        agent_bridge_joint_fixed_params_;
    intrinsic::icon::TareForceTorqueSensorInfo::FixedParams
        tare_ft_sensor_fixed_params_;
    std::optional<Action> agent_bridge_action_;
    std::optional<Action> agent_bridge_joint_action_;
    std::optional<Action> tare_action_;
    std::unique_ptr<intrinsic::icon::StreamWriterInterface<
        intrinsic_proto::icon::actions::proto::MotionUpdate>>
        agent_bridge_writer_;
    std::unique_ptr<intrinsic::icon::StreamWriterInterface<
        intrinsic_proto::icon::actions::proto::JointMotionUpdate>>
        agent_bridge_joint_writer_;

    // Intrinsic pubsub
    std::shared_ptr<intrinsic::PubSub> pubsub_;
    std::shared_ptr<intrinsic::Subscription> agent_bridge_output_stream_sub_;
    std::shared_ptr<intrinsic::Subscription>
        agent_bridge_joint_output_stream_sub_;
    std::shared_ptr<intrinsic::Subscription> robot_state_sub_;

    rclcpp::Subscription<aic_control_interfaces::msg::MotionUpdate>::SharedPtr
        motion_update_sub_;
    rclcpp::Subscription<aic_control_interfaces::msg::JointMotionUpdate>::
        SharedPtr joint_motion_update_sub_;

    rclcpp::Publisher<aic_control_interfaces::msg::ControllerState>::SharedPtr
        controller_state_pub_;

    rclcpp::Service<aic_control_interfaces::srv::ChangeTargetMode>::SharedPtr
        change_target_mode_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restart_bridge_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_ft_sensor_srv_;

    rclcpp::TimerBase::SharedPtr controller_state_timer_;

    bool connected_to_controller_;
    std::string part_name_;
    std::string ft_sensor_part_name_;
    std::string instance_;
    std::string server_address_;
    std::size_t num_joints_;
    uint8_t target_mode_value_;
    std::optional<int64_t> last_part_status_timestamp_ns_;

    aic_control_interfaces::msg::ControllerState controller_state_;
    std::mutex controller_state_mutex_;

    Data();
    ~Data();
  };
  std::shared_ptr<Data> data_;
};
}  // namespace flowstate_ros_bridge.

#endif  // BRIDGES_ROBOT_CONTROL_BRIDGE_HPP_
