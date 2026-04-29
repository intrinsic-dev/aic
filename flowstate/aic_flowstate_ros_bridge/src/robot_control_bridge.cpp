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

#include "robot_control_bridge.hpp"

#include <filesystem>
#include <string>
#include <utility>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/platform/pubsub/zenoh_util/zenoh_config.h"
#include "intrinsic/util/grpc/channel.h"
#include "intrinsic/util/grpc/connection_params.h"
#include "intrinsic/util/proto/get_text_proto.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/create_service.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/parameter.hpp"

// Interfaces
#include "aic_control_interfaces/msg/controller_state.hpp"

using namespace std::chrono_literals;

namespace flowstate_ros_bridge {
constexpr const char* kServerAddressParamName = "server_address";
constexpr const char* kInstanceParamName = "instance";
constexpr const char* kPartNameParamName = "part_name";
constexpr const char* kFTPartNameParamName = "ft_sensor_part_name";
constexpr const char* kAgentBridgeTaskSettingsFileParamName =
    "task_settings_file";
constexpr const char* kAgentBridgeJointTaskSettingsFileParamName =
    "joint_task_settings_file";
constexpr const char* kFlowstateZenohRouterParamName =
    "flowstate_zenoh_router_address";

///=============================================================================
void RobotControlBridge::declare_ros_parameters(
    ROSNodeInterfaces ros_node_interfaces) {
  const auto& param_interface =
      ros_node_interfaces
          .get<rclcpp::node_interfaces::NodeParametersInterface>();

  param_interface->declare_parameter(kServerAddressParamName,
                                     rclcpp::ParameterValue{"localhost:17080"});
  param_interface->declare_parameter(
      kInstanceParamName, rclcpp::ParameterValue{"robot_controller"});
  param_interface->declare_parameter(kPartNameParamName,
                                     rclcpp::ParameterValue{"arm"});
  param_interface->declare_parameter(kFTPartNameParamName,
                                     rclcpp::ParameterValue{"ft_sensor"});
  param_interface->declare_parameter(kAgentBridgeTaskSettingsFileParamName,
                                     rclcpp::ParameterValue{""});
  param_interface->declare_parameter(kAgentBridgeJointTaskSettingsFileParamName,
                                     rclcpp::ParameterValue{""});
}

///=============================================================================
bool RobotControlBridge::initialize(
    ROSNodeInterfaces ros_node_interfaces,
    std::shared_ptr<Executive> /*executive_client*/,
    std::shared_ptr<World> /*world_client*/) {
  data_ = std::make_shared<Data>();

  data_->node_interfaces_ = std::move(ros_node_interfaces);

  const auto& param_interface =
      data_->node_interfaces_
          .get<rclcpp::node_interfaces::NodeParametersInterface>();

  data_->server_address_ =
      param_interface->get_parameter(kServerAddressParamName)
          .get_value<std::string>();
  data_->instance_ = param_interface->get_parameter(kInstanceParamName)
                         .get_value<std::string>();
  data_->part_name_ = param_interface->get_parameter(kPartNameParamName)
                          .get_value<std::string>();
  data_->ft_sensor_part_name_ =
      param_interface->get_parameter(kFTPartNameParamName)
          .get_value<std::string>();
  std::filesystem::path task_settings_file =
      param_interface->get_parameter(kAgentBridgeTaskSettingsFileParamName)
          .get_value<std::string>();
  std::filesystem::path joint_task_settings_file =
      param_interface->get_parameter(kAgentBridgeJointTaskSettingsFileParamName)
          .get_value<std::string>();

  // Prepend share directory of current package if task_settings filepath is
  // relative
  std::filesystem::path share_dir =
      ament_index_cpp::get_package_share_directory("aic_flowstate_ros_bridge");
  if (task_settings_file.is_relative()) {
    LOG(INFO) << "'task_settings_file' parameter has value of '"
              << task_settings_file.string()
              << "' which is a relative path. "
                 "Resolving it relative to shared directory of "
                 "package 'aic_flowstate_ros_bridge'";
    task_settings_file = share_dir / task_settings_file;
  }
  if (joint_task_settings_file.is_relative()) {
    LOG(INFO) << "'joint_task_settings_file' parameter has value of '"
              << joint_task_settings_file.string()
              << "' which is a relative path. "
                 "Resolving it relative to shared directory of "
                 "package 'aic_flowstate_ros_bridge'";
    joint_task_settings_file = share_dir / joint_task_settings_file;
  }

  // Load TaskSettings from file
  if (!task_settings_file.empty()) {
    auto status = intrinsic::GetTextProto(
        task_settings_file.string(),
        *(data_->agent_bridge_fixed_params_.mutable_task_settings()));
    if (!status.ok()) {
      LOG(ERROR) << "Failed to start load task settings for AgentBridge from "
                    "filepath '"
                 << task_settings_file << "': " << status.message().data();
      throw std::runtime_error("Failed to load task settings for AgentBridge");
    }
  } else {
    LOG(ERROR) << "'task_settings_file' parameter is empty. Provide a valid "
                  "filepath for loading default task_settings.";
    throw std::runtime_error("'task_settings_file' parameter is empty.");
  }
  if (!joint_task_settings_file.empty()) {
    auto status = intrinsic::GetTextProto(
        joint_task_settings_file.string(),
        *(data_->agent_bridge_joint_fixed_params_.mutable_task_settings()));
    if (!status.ok()) {
      LOG(ERROR) << "Failed to start load task settings for AgentBridgeJoint "
                    "from filepath '"
                 << task_settings_file << "': " << status.message().data();
      throw std::runtime_error(
          "Failed to load task settings for AgentBridgeJoint");
    }
  } else {
    LOG(ERROR)
        << "'joint_task_settings_file' parameter is empty. Provide a valid "
           "filepath for loading default task_settings.";
    throw std::runtime_error("'joint_task_settings_file' parameter is empty.");
  }
  // Set default taring cycle of 100
  // todo(johntgz) set this as a ros parameter
  data_->tare_ft_sensor_fixed_params_.set_num_taring_cycles(100);

  // Create Zenoh subscriptions to the action output streams from the robot
  // controller server
  absl::SetFlag(&FLAGS_zenoh_router,
                param_interface->get_parameter(kFlowstateZenohRouterParamName)
                    .get_value<std::string>());
  data_->pubsub_ = std::make_shared<intrinsic::PubSub>(
      data_->node_interfaces_.get<rclcpp::node_interfaces::NodeBaseInterface>()
          ->get_name());

  auto agent_bridge_output_stream_sub = CreateActionOutputStreamSubscription(
      [this](const intrinsic_proto::data_logger::LogItem& msg) {
        this->agentBridgeOutputStreamCallback(msg);
      },
      data_->instance_, kAgentBridgeId);
  if (!agent_bridge_output_stream_sub.ok()) {
    LOG(ERROR) << "Unable to create subscription to action output stream for "
                  "AgentBridge: "
               << agent_bridge_output_stream_sub.status();
    throw std::runtime_error(
        "Unable to create subscription to action output stream for "
        "AgentBridge");
  }
  LOG(INFO) << "Subscribed to AgentBridge output stream topic";
  data_->agent_bridge_output_stream_sub_ =
      std::move(*agent_bridge_output_stream_sub);

  auto agent_bridge_joint_output_stream_sub =
      CreateActionOutputStreamSubscription(
          [this](const intrinsic_proto::data_logger::LogItem& msg) {
            this->agentBridgeOutputStreamCallback(msg);
          },
          data_->instance_, kAgentBridgeJointId);
  if (!agent_bridge_joint_output_stream_sub.ok()) {
    LOG(ERROR) << "Unable to create subscription to action output stream for "
                  "AgentBridgeJoint: "
               << agent_bridge_joint_output_stream_sub.status();
    throw std::runtime_error(
        "Unable to create subscription to action output stream for "
        "AgentBridgeJoint");
  }
  LOG(INFO) << "Subscribed to AgentBridgeJoint output stream topic";
  data_->agent_bridge_joint_output_stream_sub_ =
      std::move(*agent_bridge_joint_output_stream_sub);

  // Create a Zenoh subscriber on the robot_state pubsub topic
  auto robot_state_sub = CreateRobotStateSubscription(
      [this](const intrinsic_proto::data_logger::LogItem& msg) {
        this->RobotStateCallback(msg);
      },
      data_->instance_);
  if (!robot_state_sub.ok()) {
    LOG(ERROR) << "Unable to create Robot State Subscription: "
               << robot_state_sub.status();
    throw std::runtime_error("Unable to create Robot State Subscription");
  }
  LOG(INFO) << "Subscribed to Flowstate Robot State topic";
  data_->robot_state_sub_ = std::move(*robot_state_sub);

  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface>
      topics_interface =
          data_->node_interfaces_
              .get<rclcpp::node_interfaces::NodeTopicsInterface>();
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base_interface =
      data_->node_interfaces_.get<rclcpp::node_interfaces::NodeBaseInterface>();
  auto clock = data_->node_interfaces_
                   .get<rclcpp::node_interfaces::NodeClockInterface>()
                   ->get_clock();

  // Reliable QoS subscriptions for motion commands.
  rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  // ROS Subscriptions to MotionUpdate and JointMotionUpdate commands
  data_->motion_update_sub_ =
      rclcpp::create_subscription<aic_control_interfaces::msg::MotionUpdate>(
          topics_interface, "aic_controller/pose_commands", reliable_qos,
          [this](
              const aic_control_interfaces::msg::MotionUpdate::SharedPtr msg) {
            this->MotionUpdateCallback(msg);
          });
  data_->joint_motion_update_sub_ = rclcpp::create_subscription<
      aic_control_interfaces::msg::JointMotionUpdate>(
      topics_interface, "aic_controller/joint_commands", reliable_qos,
      [this](
          const aic_control_interfaces::msg::JointMotionUpdate::SharedPtr msg) {
        this->JointMotionUpdateCallback(msg);
      });

  // ROS Service server for ChangeTargetMode
  data_->change_target_mode_srv_ =
      rclcpp::create_service<aic_control_interfaces::srv::ChangeTargetMode>(
          base_interface,
          data_->node_interfaces_
              .get<rclcpp::node_interfaces::NodeServicesInterface>(),
          "aic_controller/change_target_mode",
          [this](const std::shared_ptr<
                     aic_control_interfaces::srv::ChangeTargetMode::Request>
                     request,
                 std::shared_ptr<
                     aic_control_interfaces::srv::ChangeTargetMode::Response>
                     response) {
            this->ChangeTargetModeCallback(request, response);
          },
          rclcpp::ServicesQoS(), nullptr);

  // ROS Service server for RestartBridge
  data_->restart_bridge_srv_ = rclcpp::create_service<std_srvs::srv::Trigger>(
      base_interface,
      data_->node_interfaces_
          .get<rclcpp::node_interfaces::NodeServicesInterface>(),
      "aic_controller/restart_bridge",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->RestartBridgeCallback(request, response);
      },
      rclcpp::ServicesQoS(), nullptr);

  // ROS Service server for TareForceTorqueSensor
  data_->tare_ft_sensor_srv_ = rclcpp::create_service<std_srvs::srv::Trigger>(
      base_interface,
      data_->node_interfaces_
          .get<rclcpp::node_interfaces::NodeServicesInterface>(),
      "aic_controller/tare_force_torque_sensor",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->TareForceTorqueSensorCallback(request, response);
      },
      rclcpp::ServicesQoS(), nullptr);

  // Publisher for controller state message
  data_->controller_state_pub_ =
      rclcpp::create_publisher<aic_control_interfaces::msg::ControllerState>(
          topics_interface, "aic_controller/controller_state",
          rclcpp::SystemDefaultsQoS());

  // Timer to publish controller state at 250 Hz
  data_->controller_state_timer_ = rclcpp::create_timer(
      base_interface,
      data_->node_interfaces_
          .get<rclcpp::node_interfaces::NodeTimersInterface>(),
      data_->node_interfaces_
          .get<rclcpp::node_interfaces::NodeClockInterface>()
          ->get_clock(),
      4ms, [this]() {
        std::lock_guard<std::mutex> lock(data_->controller_state_mutex_);
        data_->controller_state_.header.stamp =
            data_->node_interfaces_
                .get<rclcpp::node_interfaces::NodeClockInterface>()
                ->get_clock()
                ->now();
        data_->controller_state_.target_mode.mode = data_->target_mode_value_;
        data_->controller_state_pub_->publish(data_->controller_state_);
      });

  LOG(INFO) << "Connecting to ICON server: " << data_->server_address_
            << ", instance: " << data_->instance_
            << ", part: " << data_->part_name_;

  data_->connected_to_controller_ = restartControllerBridge();
  if (!data_->connected_to_controller_) {
    LOG(ERROR) << "Failed to connect to controller bridge.";
    // todo(johntgz) add retry mechanism
  }

  LOG(INFO) << "Initialized RobotControlBridge.";

  return true;
}

///=============================================================================
absl::StatusOr<std::shared_ptr<intrinsic::Subscription>>
RobotControlBridge::CreateActionOutputStreamSubscription(
    intrinsic::SubscriptionOkCallback<intrinsic_proto::data_logger::LogItem>
        callback,
    const std::string& instance,
    const intrinsic::icon::ActionInstanceId action_instance_id) {
  const std::string action_output_stream_topic_name =
      absl::StrFormat("/icon/%s/output_streams/action_%s", instance,
                      std::to_string(action_instance_id.value()));

  auto sub = data_->pubsub_->CreateSubscription(
      action_output_stream_topic_name, intrinsic::TopicConfig(), callback);
  if (!sub.ok()) {
    return sub.status();
  }
  return std::make_shared<intrinsic::Subscription>(std::move(*sub));
}

///=============================================================================
absl::StatusOr<std::shared_ptr<intrinsic::Subscription>>
RobotControlBridge::CreateRobotStateSubscription(
    intrinsic::SubscriptionOkCallback<intrinsic_proto::data_logger::LogItem>
        callback,
    const std::string& robot_controller_instance) {
  const std::string topic_name =
      absl::StrFormat("/icon/%s/robot_status", robot_controller_instance);

  auto sub = data_->pubsub_->CreateSubscription(
      topic_name, intrinsic::TopicConfig(), callback);
  if (!sub.ok()) {
    return sub.status();
  }
  return std::make_shared<intrinsic::Subscription>(std::move(*sub));
}

///=============================================================================
void RobotControlBridge::agentBridgeOutputStreamCallback(
    const intrinsic_proto::data_logger::LogItem& log_item) {
  auto clock = data_->node_interfaces_
                   .get<rclcpp::node_interfaces::NodeClockInterface>()
                   ->get_clock();
  const rclcpp::Time t_start = clock->now();

  const auto& payload = log_item.payload();

  intrinsic_proto::icon::StreamingOutputWithMetadata stream_out_with_meta_proto;
  if (!payload.any().UnpackTo(&stream_out_with_meta_proto)) {
    LOG(WARNING) << "Failed to unpack StreamingOutputWithMetadata from "
                    "action output stream";
    return;
  }

  std::lock_guard<std::mutex> lock(data_->controller_state_mutex_);

  switch (stream_out_with_meta_proto.action_instance_id()) {
    case kAgentBridgeId.value(): {
      intrinsic_proto::icon::actions::proto::AgentBridgeStatus ab_status_proto;
      if (!stream_out_with_meta_proto.output().payload().UnpackTo(
              &ab_status_proto)) {
        LOG(WARNING) << "Failed to unpack AgentBridgeStatus from "
                        "AgentBridgeAction streaming output";
        return;
      }

      const auto& last_update = ab_status_proto.last_control_update();
      if (last_update.reference_pose_size() == 7) {
        data_->controller_state_.reference_tcp_pose.position.x =
            last_update.reference_pose(0);
        data_->controller_state_.reference_tcp_pose.position.y =
            last_update.reference_pose(1);
        data_->controller_state_.reference_tcp_pose.position.z =
            last_update.reference_pose(2);
        data_->controller_state_.reference_tcp_pose.orientation.x =
            last_update.reference_pose(3);
        data_->controller_state_.reference_tcp_pose.orientation.y =
            last_update.reference_pose(4);
        data_->controller_state_.reference_tcp_pose.orientation.z =
            last_update.reference_pose(5);
        data_->controller_state_.reference_tcp_pose.orientation.w =
            last_update.reference_pose(6);
      }

      for (int i = 0; i < 6 && i < ab_status_proto.pose_error_integrated_size();
           ++i) {
        data_->controller_state_.tcp_error[i] =
            ab_status_proto.pose_error_integrated(i);
      }

      break;
    }
    case kAgentBridgeJointId.value(): {
      intrinsic_proto::icon::actions::proto::AgentBridgeJointStatus
          ab_joint_status_proto;
      if (!stream_out_with_meta_proto.output().payload().UnpackTo(
              &ab_joint_status_proto)) {
        LOG(WARNING) << "Failed to unpack AgentBridgeJointStatus from "
                        "AgentBridgeJointAction streaming output";
        return;
      }

      const auto& reference_state = ab_joint_status_proto.reference_state();
      data_->controller_state_.reference_joint_state.positions.clear();
      for (double pos : reference_state.position()) {
        data_->controller_state_.reference_joint_state.positions.push_back(pos);
      }
      data_->controller_state_.reference_joint_state.velocities.clear();
      for (double vel : reference_state.velocity()) {
        data_->controller_state_.reference_joint_state.velocities.push_back(
            vel);
      }
      data_->controller_state_.reference_joint_state.accelerations.clear();
      for (double acc : reference_state.acceleration()) {
        data_->controller_state_.reference_joint_state.accelerations.push_back(
            acc);
      }
      const auto& joint_torque = ab_joint_status_proto.joint_torque();
      data_->controller_state_.reference_joint_state.effort.clear();
      for (double torque : joint_torque.joints()) {
        data_->controller_state_.reference_joint_state.effort.push_back(torque);
      }

      break;
    }
    default:
      LOG(WARNING) << "Received streaming output for unknown action instance: "
                   << stream_out_with_meta_proto.action_instance_id();
      return;
  }

  // print a timing snapshot every 5000 messages
  const rclcpp::Duration elapsed = clock->now() - t_start;
  LOG_EVERY_N(INFO, 5000) << absl::StrFormat(
      "AgentBridge output stream translation time: %.3f ms",
      1000.0 * elapsed.seconds());
}

///=============================================================================
void RobotControlBridge::RobotStateCallback(
    const intrinsic_proto::data_logger::LogItem& log_item) {
  std::lock_guard<std::mutex> lock(data_->controller_state_mutex_);

  const intrinsic_proto::icon::RobotStatus& icon_robot_status =
      log_item.payload().icon_robot_status();

  auto it = icon_robot_status.status_map().find(data_->part_name_);
  if (it != icon_robot_status.status_map().end()) {
    const intrinsic_proto::icon::PartStatus& part_status = it->second;

    const auto& base_t_tip_sensed = part_status.base_t_tip_sensed();
    data_->controller_state_.tcp_pose.position.x = base_t_tip_sensed.pos().x();
    data_->controller_state_.tcp_pose.position.y = base_t_tip_sensed.pos().y();
    data_->controller_state_.tcp_pose.position.z = base_t_tip_sensed.pos().z();
    data_->controller_state_.tcp_pose.orientation.x =
        base_t_tip_sensed.rot().qx();
    data_->controller_state_.tcp_pose.orientation.y =
        base_t_tip_sensed.rot().qy();
    data_->controller_state_.tcp_pose.orientation.z =
        base_t_tip_sensed.rot().qz();
    data_->controller_state_.tcp_pose.orientation.w =
        base_t_tip_sensed.rot().qw();

    const auto& base_twist_tip_sensed = part_status.base_twist_tip_sensed();
    data_->controller_state_.tcp_velocity.linear.x = base_twist_tip_sensed.x();
    data_->controller_state_.tcp_velocity.linear.y = base_twist_tip_sensed.y();
    data_->controller_state_.tcp_velocity.linear.z = base_twist_tip_sensed.z();
    data_->controller_state_.tcp_velocity.angular.x =
        base_twist_tip_sensed.rx();
    data_->controller_state_.tcp_velocity.angular.y =
        base_twist_tip_sensed.ry();
    data_->controller_state_.tcp_velocity.angular.z =
        base_twist_tip_sensed.rz();

  } else {
    LOG_EVERY_N(ERROR, 100)
        << "Error: Unable to find robot part [" << data_->part_name_
        << "] within robot_status message!";
  }

  const auto& current_timestamp_ns = icon_robot_status.timestamp_ns();

  if (data_->last_part_status_timestamp_ns_.has_value() &&
      current_timestamp_ns < data_->last_part_status_timestamp_ns_.value()) {
    LOG(WARNING) << "Backwards time jump detected for controller server "
                    "timestamp, indicating a reset.";
    data_->connected_to_controller_ = false;
  }
  data_->last_part_status_timestamp_ns_ = current_timestamp_ns;
}

///=============================================================================
void RobotControlBridge::RestartBridgeCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  LOG(INFO) << "Received request to restart controller bridge";

  if (!restartControllerBridge()) {
    response->success = false;
    response->message = "Failed to restart controller bridge";
    return;
  }

  response->success = true;
  response->message = "Successfully restarted controller bridge";
}

///=============================================================================
void RobotControlBridge::TareForceTorqueSensorCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  LOG(INFO) << "Received request to tare force torque sensor";

  if (!data_->connected_to_controller_) {
    LOG(INFO) << "Restarting connection to controller server";
    data_->connected_to_controller_ = restartControllerBridge();
  }

  if (!data_->connected_to_controller_ || !data_->session_ ||
      !data_->tare_action_.has_value()) {
    response->success = false;
    response->message =
        "No active controller session or tare action not initialized";
    return;
  }

  // todo(johntgz) should we Start the tare action in parallel with existing
  // motion plans, we by setting stop_active_actions = false
  auto status =
      data_->session_->StartAction(data_->tare_action_.value(), false);
  if (!status.ok()) {
    LOG(ERROR) << "Failed to start TareForceTorqueSensor Action: "
               << status.message().data();
    response->success = false;
    response->message = "Failed to start TareForceTorqueSensor Action";
    return;
  }

  LOG(INFO) << "Successfully started TareForceTorqueSensor Action";
  response->success = true;
  response->message = "Successfully started TareForceTorqueSensor Action";
}

///=============================================================================
bool RobotControlBridge::restartControllerBridge() {
  // Stop all currently running actions
  if (data_->session_) {
    auto status = data_->session_->StopAllActions();
    if (!status.ok()) {
      LOG(ERROR) << "Failed to stop all actions on ICON Server: "
                 << status.message().data();
    }
  }

  // Reset client, session, actions and stream writers
  data_->session_.reset();
  data_->agent_bridge_action_ = std::nullopt;
  data_->agent_bridge_joint_action_ = std::nullopt;
  data_->tare_action_ = std::nullopt;
  data_->agent_bridge_writer_.reset();
  data_->agent_bridge_joint_writer_.reset();

  if (!startControllerSession()) {
    LOG(ERROR) << "Failed to start ICON client and session.";
    return false;
  }

  if (!resetMotionUpdate()) {
    LOG(ERROR) << "Failed to reset MotionUpdate and JointMotionUpdate "
                  "commands.";
    return false;
  }

  if (!startControllerAction()) {
    LOG(ERROR) << "Failed to add and start AgentBridge actions.";
    return false;
  }

  LOG(INFO) << "Successfully established a connection to the "
               "controller server!";
  return true;
}

///=============================================================================
bool RobotControlBridge::startControllerAction() {
  if (!data_->session_) {
    LOG(ERROR) << "A session has not been created. Unable to add actions.";
    return false;
  }

  // Define ActionDescriptors for both the AgentBridge and AgentBridgeJoint
  // actions
  ActionDescriptor agent_bridge_descriptor =
      ActionDescriptor(
          intrinsic::icon::AgentBridgeInfo::kActionTypeName, kAgentBridgeId,
          {{intrinsic::icon::AgentBridgeInfo::kSlotName, data_->part_name_}})
          .WithFixedParams(data_->agent_bridge_fixed_params_);

  ActionDescriptor agent_bridge_joint_descriptor =
      ActionDescriptor(intrinsic::icon::AgentBridgeJointInfo::kActionTypeName,
                       kAgentBridgeJointId,
                       {{intrinsic::icon::AgentBridgeJointInfo::kSlotName,
                         data_->part_name_}})
          .WithFixedParams(data_->agent_bridge_joint_fixed_params_);

  ActionDescriptor tare_ft_sensor_descriptor =
      ActionDescriptor(
          intrinsic::icon::TareForceTorqueSensorInfo::kActionTypeName,
          kTareForceTorqueSensorId,
          {{intrinsic::icon::TareForceTorqueSensorInfo::
                kForceTorqueSensorSlotName,
            data_->ft_sensor_part_name_}})
          .WithFixedParams(data_->tare_ft_sensor_fixed_params_);

  // Add the AgentBridge and AgentBridgeJoint actions to the session, then
  // create StreamWriters for each of the actions.

  // Add the AgentBridge action to the current session
  auto agent_bridge_action_or =
      data_->session_->AddAction(agent_bridge_descriptor);
  if (!agent_bridge_action_or.ok()) {
    LOG(ERROR) << "Failed to add AgentBridge Action: "
               << agent_bridge_action_or.status().message().data();
    return false;
  }
  data_->agent_bridge_action_ = agent_bridge_action_or.value();

  // Add the AgentBridgeJoint action to the current session
  auto agent_bridge_joint_action_or =
      data_->session_->AddAction(agent_bridge_joint_descriptor);
  if (!agent_bridge_joint_action_or.ok()) {
    LOG(ERROR) << "Failed to add AgentBridgeJoint Action: "
               << agent_bridge_joint_action_or.status().message().data();
    return false;
  }
  data_->agent_bridge_joint_action_ = agent_bridge_joint_action_or.value();

  // Add the TareForceTorqueSensor action to the current session
  auto tare_action_or = data_->session_->AddAction(tare_ft_sensor_descriptor);
  if (!tare_action_or.ok()) {
    LOG(ERROR) << "Failed to add TareForceTorqueSensor Action: "
               << tare_action_or.status().message().data();
    return false;
  }
  data_->tare_action_ = tare_action_or.value();

  // Create StreamWriter for the AgentBridge action
  auto agent_bridge_writer_or =
      data_->session_
          ->StreamWriter<intrinsic_proto::icon::actions::proto::MotionUpdate>(
              data_->agent_bridge_action_.value(),
              intrinsic::icon::AgentBridgeInfo::kStreamingCommandName);
  if (!agent_bridge_writer_or.ok()) {
    LOG(ERROR)
        << "Failed to create MotionUpdate stream writer to AgentBridge action: "
        << agent_bridge_writer_or.status().message().data();
    return false;
  }
  data_->agent_bridge_writer_ = std::move(agent_bridge_writer_or.value());

  // Create StreamWriter for the AgentBridgeJoint action
  auto agent_bridge_joint_writer_or = data_->session_->StreamWriter<
      intrinsic_proto::icon::actions::proto::JointMotionUpdate>(
      data_->agent_bridge_joint_action_.value(),
      intrinsic::icon::AgentBridgeJointInfo::kStreamingCommandName);
  if (!agent_bridge_joint_writer_or.ok()) {
    LOG(ERROR) << "Failed to create JointMotionUpdate stream writer to "
                  "AgentBridgeJoint action: "
               << agent_bridge_joint_writer_or.status().message().data();
    return false;
  }
  data_->agent_bridge_joint_writer_ =
      std::move(agent_bridge_joint_writer_or.value());

  // The default action to start on initialization is the AgentBridge action
  auto status =
      data_->session_->StartAction(data_->agent_bridge_action_.value());
  if (!status.ok()) {
    LOG(ERROR) << "Failed to start AgentBridge Action: "
               << status.message().data();
    return false;
  }
  LOG(INFO) << "Successfully started AgentBridge Action on ICON Server";

  data_->target_mode_value_ =
      aic_control_interfaces::msg::TargetMode::MODE_CARTESIAN;

  return true;
}

///=============================================================================
bool RobotControlBridge::startControllerSession() {
  auto connection_params = intrinsic::ConnectionParams::ResourceInstance(
      data_->instance_, data_->server_address_);

  auto icon_channel_or = intrinsic::Channel::MakeFromAddress(connection_params);
  if (!icon_channel_or.ok()) {
    LOG(ERROR) << "Failed to create ICON channel: "
               << icon_channel_or.status().message().data();
    return false;
  }
  auto client = Client(icon_channel_or.value());

  // Check operation status of ICON Server
  absl::StatusOr<OperationalStatus> operational_status =
      client.GetOperationalStatus();
  if (!operational_status.ok()) {
    LOG(ERROR) << "Failed to retrieve operational status of ICON Server.";
    return false;
  }
  switch (operational_status->state()) {
    case intrinsic::icon::OperationalState::kDisabled:
      LOG(ERROR) << "ICON Server operational status: Disabled!";
      return false;
      break;
    case intrinsic::icon::OperationalState::kFaulted:
      LOG(ERROR) << "ICON Server operational status: Faulted! Reason: "
                 << operational_status->fault_reason();
      return false;
      break;
    case intrinsic::icon::OperationalState::kEnabled:
      LOG(INFO) << "ICON Server operational status: Enabled.";
      break;
    default:
      LOG(INFO) << "ICON Server operational status: Unknown.";
      return false;
  }

  // Get part status information such as number of joints
  absl::StatusOr<intrinsic_proto::icon::PartStatus> part_status =
      client.GetSinglePartStatus(data_->part_name_);
  if (!part_status.ok()) {
    LOG(ERROR) << "Failed to retrieve part status from ICON Server: "
               << part_status.status().message().data();
    return false;
  }
  data_->num_joints_ = part_status.value().joint_states_size();

  // Start ICON Session
  std::vector<std::string> parts = {data_->part_name_};
  if (data_->ft_sensor_part_name_ != data_->part_name_) {
    parts.push_back(data_->ft_sensor_part_name_);
  }
  // todo(johntgz) test if starting a session with multiple parts would cause an
  // issue
  auto session_or = Session::Start(icon_channel_or.value(), parts);
  if (!session_or.ok()) {
    LOG(ERROR) << "Failed to start ICON session: "
               << session_or.status().message().data();
    return false;
  }
  data_->session_ = std::move(session_or.value());

  LOG(INFO) << "Successfully started ICON session";

  return true;
}

///=============================================================================
bool RobotControlBridge::resetMotionUpdate() {
  // Initialize Motion Update defaults
  auto* initial_motion_update =
      data_->agent_bridge_fixed_params_.mutable_motion_update();
  if (!initial_motion_update->has_target_state()) {
    initial_motion_update->set_trajectory_generation_mode(
        intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
            VELOCITY);
    auto* target_state = initial_motion_update->mutable_target_state();
    target_state->mutable_velocity()->Resize(6, 0.0);

    // Use max values from task settings for initial stiffness, damping, and
    // mass
    auto* target_stiffness = initial_motion_update->mutable_target_stiffness();
    auto* target_damping = initial_motion_update->mutable_target_damping();
    auto* target_mass = initial_motion_update->mutable_target_mass();

    target_stiffness->mutable_data()->Resize(36, 0.0);
    target_damping->mutable_data()->Resize(36, 0.0);
    target_mass->mutable_data()->Resize(36, 0.0);

    if (data_->agent_bridge_fixed_params_.task_settings().has_max_stiffness() &&
        data_->agent_bridge_fixed_params_.task_settings().has_max_damping() &&
        data_->agent_bridge_fixed_params_.task_settings()
            .has_mass_for_critical_damping()) {
      const auto& max_stiffness =
          data_->agent_bridge_fixed_params_.task_settings().max_stiffness();
      const auto& max_damping =
          data_->agent_bridge_fixed_params_.task_settings().max_damping();
      const auto& critical_mass =
          data_->agent_bridge_fixed_params_.task_settings()
              .mass_for_critical_damping();

      target_stiffness->set_data(0, max_stiffness.x());
      target_stiffness->set_data(7, max_stiffness.y());
      target_stiffness->set_data(14, max_stiffness.z());
      target_stiffness->set_data(21, max_stiffness.rx());
      target_stiffness->set_data(28, max_stiffness.ry());
      target_stiffness->set_data(35, max_stiffness.rz());

      target_damping->set_data(0, max_damping.x());
      target_damping->set_data(7, max_damping.y());
      target_damping->set_data(14, max_damping.z());
      target_damping->set_data(21, max_damping.rx());
      target_damping->set_data(28, max_damping.ry());
      target_damping->set_data(35, max_damping.rz());

      target_mass->set_data(0, critical_mass.x());
      target_mass->set_data(7, critical_mass.y());
      target_mass->set_data(14, critical_mass.z());
      target_mass->set_data(21, critical_mass.rx());
      target_mass->set_data(28, critical_mass.ry());
      target_mass->set_data(35, critical_mass.rz());
    } else {
      LOG(ERROR) << "Task settings missing max_stiffness, max_damping, or "
                    "mass_for_critical_damping parameters.";
      return false;
    }

    auto* fw = initial_motion_update->mutable_feedforward_wrench_at_tip();
    fw->set_x(0);
    fw->set_y(0);
    fw->set_z(0);
    fw->set_rx(0);
    fw->set_ry(0);
    fw->set_rz(0);
  }

  // Initialize Joint Motion Update defaults
  if (data_->num_joints_ == 0) {
    LOG(ERROR) << "Error initializing JointMotionUpdate with num_joints_==0. ";
    return false;
  }

  auto* initial_joint_motion_update =
      data_->agent_bridge_joint_fixed_params_.mutable_motion_update();
  initial_joint_motion_update->set_trajectory_generation_mode(
      intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
          VELOCITY);
  initial_joint_motion_update->mutable_target_state()
      ->mutable_velocity()
      ->Resize(data_->num_joints_, 0.0);

  auto* target_joint_stiffness =
      initial_joint_motion_update->mutable_target_stiffness();
  auto* target_joint_damping =
      initial_joint_motion_update->mutable_target_damping();

  target_joint_stiffness->mutable_joints()->Resize(data_->num_joints_, 0.0);
  target_joint_damping->mutable_joints()->Resize(data_->num_joints_, 0.0);

  if (data_->agent_bridge_joint_fixed_params_.task_settings().has_stiffness() &&
      data_->agent_bridge_joint_fixed_params_.task_settings().has_damping()) {
    const auto& stiffness =
        data_->agent_bridge_joint_fixed_params_.task_settings().stiffness();
    const auto& damping =
        data_->agent_bridge_joint_fixed_params_.task_settings().damping();

    for (std::size_t i = 0; i < data_->num_joints_; i++) {
      target_joint_stiffness->set_joints(i, stiffness.joints(i));
      target_joint_damping->set_joints(i, damping.joints(i));
    }
  }

  return true;
}

///=============================================================================
void RobotControlBridge::ChangeTargetModeCallback(
    const std::shared_ptr<
        aic_control_interfaces::srv::ChangeTargetMode::Request>
        request,
    std::shared_ptr<aic_control_interfaces::srv::ChangeTargetMode::Response>
        response) {
  auto target_mode_to_string = [](uint8_t mode) {
    switch (mode) {
      case aic_control_interfaces::msg::TargetMode::MODE_UNSPECIFIED:
        return "MODE_UNSPECIFIED";
      case aic_control_interfaces::msg::TargetMode::MODE_CARTESIAN:
        return "MODE_CARTESIAN";
      case aic_control_interfaces::msg::TargetMode::MODE_JOINT:
        return "MODE_JOINT";
      default:
        return "UNKNOWN_MODE";
    }
  };

  LOG(INFO) << "Received request to change target mode to: "
            << target_mode_to_string(request->target_mode.mode);

  if (request->target_mode.mode == data_->target_mode_value_) {
    LOG(INFO) << "Already in target mode: "
              << target_mode_to_string(request->target_mode.mode);
    response->success = true;
    return;
  }

  auto status = data_->session_->StopAllActions();
  if (!status.ok()) {
    LOG(ERROR) << "Failed to stop all actions on ICON Server: "
               << status.message().data();
    response->success = false;
    return;
  }
  LOG(INFO) << "Stopped all actions on ICON Server.";

  if (request->target_mode.mode ==
      aic_control_interfaces::msg::TargetMode::MODE_CARTESIAN) {
    auto status =
        data_->session_->StartAction(data_->agent_bridge_action_.value());
    if (!status.ok()) {
      LOG(ERROR) << "Failed to start AgentBridge Action: "
                 << status.message().data();
      response->success = false;
      return;
    }
    LOG(INFO) << "Successfully started AgentBridge Action on ICON Server";

    data_->target_mode_value_ =
        aic_control_interfaces::msg::TargetMode::MODE_CARTESIAN;

  } else if (request->target_mode.mode ==
             aic_control_interfaces::msg::TargetMode::MODE_JOINT) {
    auto status =
        data_->session_->StartAction(data_->agent_bridge_joint_action_.value());
    if (!status.ok()) {
      LOG(ERROR) << "Failed to start AgentBridgeJoint Action: "
                 << status.message().data();
      response->success = false;
      return;
    }
    LOG(INFO) << "Successfully started AgentBridgeJoint Action on ICON Server";

    data_->target_mode_value_ =
        aic_control_interfaces::msg::TargetMode::MODE_JOINT;
  } else {
    LOG(ERROR) << "Unsupported target mode requested.";
    response->success = false;
    return;
  }

  response->success = true;
}

///=============================================================================
void RobotControlBridge::MotionUpdateCallback(
    const aic_control_interfaces::msg::MotionUpdate::SharedPtr msg) {
  if (!data_->connected_to_controller_) {
    LOG(INFO) << "Restarting connection to controller server";
    data_->connected_to_controller_ = restartControllerBridge();
    return;
  }

  if (!data_->agent_bridge_writer_) {
    LOG(ERROR) << "MotionUpdate stream writer not initialized.";
    return;
  }

  if (data_->target_mode_value_ !=
      aic_control_interfaces::msg::TargetMode::MODE_CARTESIAN) {
    LOG(ERROR) << "Controller is not in Cartesian target mode and will not "
                  "process MotionUpdate commands.";
    return;
  }

  LOG_EVERY_N(INFO, 1000) << absl::StrFormat("Received MotionUpdate command");

  // Start with the default motion update
  intrinsic_proto::icon::actions::proto::MotionUpdate proto_msg =
      data_->agent_bridge_fixed_params_.motion_update();

  // Map the trajectory generation mode to supported modes POSITION and VELOCITY
  if (msg->trajectory_generation_mode.mode ==
      aic_control_interfaces::msg::TrajectoryGenerationMode::MODE_POSITION) {
    proto_msg.set_trajectory_generation_mode(
        intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
            POSITION);
  } else if (msg->trajectory_generation_mode.mode ==
             aic_control_interfaces::msg::TrajectoryGenerationMode::
                 MODE_VELOCITY) {
    proto_msg.set_trajectory_generation_mode(
        intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
            VELOCITY);
  } else {
    LOG(WARNING)
        << "trajectory_generation_mode is unspecified in MotionUpdate command!";
    proto_msg.set_trajectory_generation_mode(
        intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
            TRAJECTORY_GENERATION_MODE_UNSPECIFIED);
  }

  // Map target pose and velocities
  auto* target_state = proto_msg.mutable_target_state();
  target_state->clear_pose();
  target_state->add_pose(msg->pose.position.x);
  target_state->add_pose(msg->pose.position.y);
  target_state->add_pose(msg->pose.position.z);
  target_state->add_pose(msg->pose.orientation.x);
  target_state->add_pose(msg->pose.orientation.y);
  target_state->add_pose(msg->pose.orientation.z);
  target_state->add_pose(msg->pose.orientation.w);

  target_state->clear_velocity();
  target_state->add_velocity(msg->velocity.linear.x);
  target_state->add_velocity(msg->velocity.linear.y);
  target_state->add_velocity(msg->velocity.linear.z);
  target_state->add_velocity(msg->velocity.angular.x);
  target_state->add_velocity(msg->velocity.angular.y);
  target_state->add_velocity(msg->velocity.angular.z);

  // Map stiffness and damping parameters
  auto* stiffness = proto_msg.mutable_target_stiffness();
  auto* damping = proto_msg.mutable_target_damping();
  stiffness->clear_data();
  damping->clear_data();
  for (int i = 0; i < 36; ++i) {
    stiffness->add_data(msg->target_stiffness[i]);
    damping->add_data(msg->target_damping[i]);
  }

  // Map feedforward wrench at tip
  auto* fw = proto_msg.mutable_feedforward_wrench_at_tip();
  fw->set_x(msg->feedforward_wrench_at_tip.force.x);
  fw->set_y(msg->feedforward_wrench_at_tip.force.y);
  fw->set_z(msg->feedforward_wrench_at_tip.force.z);
  fw->set_rx(msg->feedforward_wrench_at_tip.torque.x);
  fw->set_ry(msg->feedforward_wrench_at_tip.torque.y);
  fw->set_rz(msg->feedforward_wrench_at_tip.torque.z);

  // Map the wrench feedback gains at tip
  auto* gains = proto_msg.mutable_wrench_feedback_gains_at_tip();
  gains->set_x(msg->wrench_feedback_gains_at_tip[0]);
  gains->set_y(msg->wrench_feedback_gains_at_tip[1]);
  gains->set_z(msg->wrench_feedback_gains_at_tip[2]);
  gains->set_rx(msg->wrench_feedback_gains_at_tip[3]);
  gains->set_ry(msg->wrench_feedback_gains_at_tip[4]);
  gains->set_rz(msg->wrench_feedback_gains_at_tip[5]);

  // Set time_to_target_seconds to a default of 1.1 * controller period (0.002
  // s), so it will reach the target as fast as possible
  proto_msg.set_time_to_target_seconds(0.0022);

  // Write the MotionUpdate proto message to the AgentBridge action
  auto status = data_->agent_bridge_writer_->Write(proto_msg);
  if (!status.ok()) {
    LOG(ERROR) << "Failed to write MotionUpdate to AgentBridge: "
               << status.message().data();
  }
}

///=============================================================================
void RobotControlBridge::JointMotionUpdateCallback(
    const aic_control_interfaces::msg::JointMotionUpdate::SharedPtr msg) {
  if (!data_->connected_to_controller_) {
    LOG(INFO) << "Restarting connection to controller server";
    data_->connected_to_controller_ = restartControllerBridge();
    return;
  }

  if (!data_->agent_bridge_joint_writer_) {
    LOG(ERROR) << "JointMotionUpdate stream writer not initialized.";
    return;
  }

  if (data_->target_mode_value_ !=
      aic_control_interfaces::msg::TargetMode::MODE_JOINT) {
    LOG(ERROR) << "Controller is not in joint target mode and will not "
                  "process JointMotionUpdate commands.";
    return;
  }

  LOG_EVERY_N(INFO, 1000) << absl::StrFormat(
      "Received JointMotionUpdate command");

  intrinsic_proto::icon::actions::proto::JointMotionUpdate proto_msg =
      data_->agent_bridge_joint_fixed_params_.motion_update();

  // Map the trajectory generation mode to supported modes POSITION and VELOCITY
  if (msg->trajectory_generation_mode.mode ==
      aic_control_interfaces::msg::TrajectoryGenerationMode::MODE_POSITION) {
    proto_msg.set_trajectory_generation_mode(
        intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
            POSITION);
  } else if (msg->trajectory_generation_mode.mode ==
             aic_control_interfaces::msg::TrajectoryGenerationMode::
                 MODE_VELOCITY) {
    proto_msg.set_trajectory_generation_mode(
        intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
            VELOCITY);
  } else {
    LOG(WARNING) << "trajectory_generation_mode is unspecified in "
                    "JointMotionUpdate command!";
    proto_msg.set_trajectory_generation_mode(
        intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
            TRAJECTORY_GENERATION_MODE_UNSPECIFIED);
  }

  // Map the target positions and velocities
  auto* target_state = proto_msg.mutable_target_state();
  target_state->clear_position();
  target_state->clear_velocity();
  for (double pos : msg->target_state.positions) {
    target_state->add_position(pos);
  }
  for (double vel : msg->target_state.velocities) {
    target_state->add_velocity(vel);
  }

  // Map the stiffness and damping parameters
  auto* stiffness = proto_msg.mutable_target_stiffness();
  auto* damping = proto_msg.mutable_target_damping();
  stiffness->clear_joints();
  damping->clear_joints();
  for (double s : msg->target_stiffness) {
    stiffness->add_joints(s);
  }
  for (double d : msg->target_damping) {
    damping->add_joints(d);
  }

  // Map the feedforward torque
  auto* ff_torque = proto_msg.mutable_target_feedforward_torque();
  ff_torque->clear_joints();
  for (double t : msg->target_feedforward_torque) {
    ff_torque->add_joints(t);
  }

  // Set time_to_target_seconds to a default of 1.1 * controller period (0.002
  // s), so it will reach the target as fast as possible
  proto_msg.set_time_to_target_seconds(0.0022);

  // Write the JointMotionUpdate proto message to the AgentBridgeJoint action
  auto status = data_->agent_bridge_joint_writer_->Write(proto_msg);
  if (!status.ok()) {
    LOG(ERROR) << "Failed to write JointMotionUpdate to AgentBridgeJoint: "
               << status.message().data();
  }
}

///=============================================================================
RobotControlBridge::Data::Data()
    : connected_to_controller_{false},
      target_mode_value_{
          aic_control_interfaces::msg::TargetMode::MODE_UNSPECIFIED},
      last_part_status_timestamp_ns_{0} {}

///=============================================================================
RobotControlBridge::Data::~Data() {
  auto status = session_->StopAllActions();
  if (!status.ok()) {
    LOG(ERROR) << "Failed to stop all actions on ICON Server: "
               << status.message().data();
  }

  session_.reset();
  agent_bridge_action_ = std::nullopt;
  agent_bridge_joint_action_ = std::nullopt;
  tare_action_ = std::nullopt;
  agent_bridge_writer_.reset();
  agent_bridge_joint_writer_.reset();

  pubsub_.reset();
  agent_bridge_output_stream_sub_.reset();
  agent_bridge_joint_output_stream_sub_.reset();
  robot_state_sub_.reset();

  motion_update_sub_.reset();
  joint_motion_update_sub_.reset();
  change_target_mode_srv_.reset();
  restart_bridge_srv_.reset();
  tare_ft_sensor_srv_.reset();
  controller_state_pub_.reset();
  controller_state_timer_.reset();

  connected_to_controller_ = false;
  target_mode_value_ =
      aic_control_interfaces::msg::TargetMode::MODE_UNSPECIFIED;
  last_part_status_timestamp_ns_ = 0;
}
}  // namespace flowstate_ros_bridge

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(flowstate_ros_bridge::RobotControlBridge,
                       flowstate_ros_bridge::BridgeInterface)
