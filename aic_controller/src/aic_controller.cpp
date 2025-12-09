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

#include "aic_controller/aic_controller.hpp"

//==============================================================================
namespace {

//==============================================================================
// called from RT control loop
void reset_motion_update_msg(aic_controller::MotionUpdate& msg) {
  msg = aic_controller::MotionUpdate();
}

}  // namespace anonymous

//==============================================================================
namespace aic_controller {

//==============================================================================
Controller::Controller()
    : param_listener_(nullptr),
      num_joints_(0),
      control_mode_(ControlMode::Invalid),
      cartesian_impedance_action_(nullptr),
      motion_update_sub_(nullptr),
      motion_update_received_(false),
      last_commanded_state_(std::nullopt),
      target_state_(std::nullopt),
      time_to_target_seconds_(0.0),
      remaining_time_to_target_seconds_(0.0) {
  // Do nothing.
}

//==============================================================================
controller_interface::InterfaceConfiguration
Controller::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  std::vector<std::string> command_interfaces_config_names;

  std::string controller_prefix, interface;
  if (control_mode_ == ControlMode::Admittance) {
    // Only initialize position interfaces in admittance control mode
    interface = hardware_interface::HW_IF_POSITION;
    controller_prefix = params_.admittance_controller_namespace + "/";
  } else if (control_mode_ == ControlMode::Impedance) {
    // Only initialize effort interfaces in impedance control mode
    interface = hardware_interface::HW_IF_EFFORT;
    controller_prefix = "";
  }

  for (const auto& joint : params_.joints) {
    command_interfaces_config_names.push_back(controller_prefix + joint + "/" + interface);
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          command_interfaces_config_names};
}

//==============================================================================
controller_interface::InterfaceConfiguration
Controller::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  std::vector<std::string> state_interfaces_config_names;

  // Add position and velocity state interfaces
  for (const auto& joint : params_.joints) {
    state_interfaces_config_names.push_back(
      joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto& joint : params_.joints) {
    state_interfaces_config_names.push_back(
      joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          state_interfaces_config_names};
}

//==============================================================================
controller_interface::CallbackReturn Controller::on_init() {
  try {
    param_listener_ =
        std::make_shared<aic_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Validate number of joints
  num_joints_ = params_.joints.size();
  if (num_joints_ < 1) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Number of joints must be > 0. provided num_joints is %ld",
                 num_joints_);
    return controller_interface::CallbackReturn::ERROR;
  }

  cartesian_impedance_action_ =
      std::make_unique<CartesianImpedanceAction>(num_joints_);

  return controller_interface::CallbackReturn::SUCCESS;
}

//==============================================================================
controller_interface::CallbackReturn Controller::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (params_.control_mode == "impedance") {
    RCLCPP_INFO(get_node()->get_logger(), "Control mode set to impedance");
    control_mode_ = ControlMode::Impedance;
  } else if (params_.control_mode == "admittance") {
    RCLCPP_INFO(get_node()->get_logger(), "Control mode set to admittance");
    control_mode_ = ControlMode::Admittance;
  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unsupported control mode. Please set control_mode to either "
                 "'admittance' or 'impedance'");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Reliable QoS subscriptions for motion updates.
  rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  motion_update_sub_ = this->get_node()->create_subscription<MotionUpdate>(
      "~/motion_update", reliable_qos,
      [this](const MotionUpdate::SharedPtr msg) {
        if (get_node()->get_current_state().id() !=
          lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){
          RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
                               *get_node()->get_clock(),
                               1000,
                               "Controller is not in ACTIVE lifecycle state, "
                               "ignoring MotionUpdate message.");

          return;
        }

        motion_update_rt_.set(*msg);
        motion_update_received_ = true;
      });

  if (!cartesian_impedance_action_->Configure(
          get_node(), this->get_robot_description())) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

//==============================================================================
controller_interface::CallbackReturn Controller::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // read and initialize current joint states
  current_state_.positions.assign(num_joints_, 0.0);
  current_state_.velocities.assign(num_joints_, 0.0);
  read_state_from_hardware(current_state_);
  for (const auto& val : current_state_.positions) {
    if (std::isnan(val)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to read joint positions from the hardware.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  reset_motion_update_msg(motion_update_);
  motion_update_rt_.try_set(motion_update_);

  return controller_interface::CallbackReturn::SUCCESS;
}

//==============================================================================
controller_interface::CallbackReturn Controller::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  release_interfaces();

  reset_motion_update_msg(motion_update_);
  motion_update_rt_.try_set(motion_update_);

  return controller_interface::CallbackReturn::SUCCESS;
}

//==============================================================================
controller_interface::CallbackReturn Controller::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  param_listener_.reset();
  motion_update_sub_.reset();

  last_commanded_state_ = std::nullopt;
  target_state_ = std::nullopt;

  time_to_target_seconds_ = 0.0;
  remaining_time_to_target_seconds_ = 0.0;

  return controller_interface::CallbackReturn::SUCCESS;
}

//==============================================================================
controller_interface::return_type Controller::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Read and update current states from sensors
  read_state_from_hardware(current_state_);

  // read user commands
  if (motion_update_received_){
    auto command_op = motion_update_rt_.try_get();
    if (command_op.has_value()) {
      motion_update_ = command_op.value();
      target_state_ = CartesianState(motion_update_.pose, motion_update_.velocity);
    }
  }

  if (!target_state_.has_value()){
    // If target_state_ has no value, return early as there is nothing
    // to write to the hardware interfaces.
    return controller_interface::return_type::OK;
  }

  JointTrajectoryPoint new_reference;

  // Clamp the target states to stay within limits
  switch (motion_update_.trajectory_generation_mode.mode) {
    case TrajectoryGenerationMode::MODE_POSITION:
      // UNIMPLEMENTED
      // Clamp poses to limit
      break;
    case TrajectoryGenerationMode::MODE_VELOCITY:
      // UNIMPLEMENTED
      // Clamp twist to limit
      RCLCPP_ERROR(get_node()->get_logger(),
                   "MODE_VELOCITY trajectory generation mode is unimplemented. "
                   "Please use MODE_POSITION.");
      return controller_interface::return_type::ERROR;

      break;
    case TrajectoryGenerationMode::MODE_POSITION_AND_VELOCITY:
      // UNIMPLEMENTED
      // Clamp pose and twist to limit
      RCLCPP_ERROR(get_node()->get_logger(),
                   "MODE_POSITION_AND_VELOCITY trajectory generation mode is "
                   "unimplemented. Please use MODE_POSITION.");
      return controller_interface::return_type::ERROR;

      break;
    default:
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Unsupported trajectory generation mode. Please set to "
                   "either MODE_POSITION, MODE_VELOCITY or "
                   "MODE_POSITION_AND_VELOCITY");
      return controller_interface::return_type::ERROR;
  }

  time_to_target_seconds_ = motion_update_.time_to_target_seconds;

  // UNIMPLEMENTED
  // Apply linear interpolation to the target_state_ to obtain a new
  // reference. Linear interpolation should support MODE_POSITION,
  // MODE_VELOCITY and MODE_POSITION_AND_VELOCITY

  if (control_mode_ == ControlMode::Impedance) {
    // UNIMPLEMENTED
    // Interpolate impedance parameters and feed-forward wrench
    // Compute control torques

    new_reference = cartesian_impedance_action_->Compute(
      target_state_.value(), current_state_);

    RCLCPP_ERROR(get_node()->get_logger(),
                 "Impedance control is unimplemented.");

    return controller_interface::return_type::ERROR;
  } else if (control_mode_ == ControlMode::Admittance) {
    // UNIMPLEMENTED
    // Perform IK to get joint targets from target_state_
  }

  write_state_to_hardware(new_reference);

  return controller_interface::return_type::OK;
}

//==============================================================================
void Controller::read_state_from_hardware(JointTrajectoryPoint& state_current) {
  // Set state_current to last commanded state if any of the hardware interface
  // values are NaN
  bool nan_position = false;
  bool nan_velocity = false;

  for (std::size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind) {
    const auto state_current_position_op =
        state_interfaces_[num_joints_ + joint_ind].get_optional();
    nan_position |= !state_current_position_op.has_value() ||
                    std::isnan(state_current_position_op.value());
    if (state_current_position_op.has_value()) {
      state_current.positions[joint_ind] = state_current_position_op.value();
    }

    auto state_current_velocity_op =
        state_interfaces_[num_joints_ + joint_ind].get_optional();
    nan_velocity |= !state_current_velocity_op.has_value() ||
                    std::isnan(state_current_velocity_op.value());

    if (state_current_velocity_op.has_value()) {
      state_current.velocities[joint_ind] = state_current_velocity_op.value();
    }
  }

  if (nan_position) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Read NaN value from position state interface, setting "
                 "current position to last_commanded_state_");
    if (last_commanded_state_.has_value()) {
      state_current.positions = last_commanded_state_.value().positions;
    }
  }
  if (nan_velocity) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Read NaN value from velocity state interface, setting "
                 "current velocity to last_commanded_state_");
    if (last_commanded_state_.has_value()) {
      state_current.velocities = last_commanded_state_.value().velocities;
    }
  }
}

//==============================================================================
void Controller::write_state_to_hardware(
    const JointTrajectoryPoint& state_commanded) {
  for (std::size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind) {
    bool success = true;

    if (control_mode_ == ControlMode::Admittance) {
      // Only write position commands in admittance control mode
      success &= command_interfaces_[joint_ind].set_value(
          state_commanded.positions[joint_ind]);
    } else if (control_mode_ == ControlMode::Impedance) {
      // Only write effort commands in impedance control mode
      success &= command_interfaces_[joint_ind].set_value(
          state_commanded.effort[joint_ind]);
    }

    if (!success) {
      RCLCPP_WARN(this->get_node()->get_logger(),
                  "Error while setting command for joint %zu.", joint_ind);
    }
  }

  last_commanded_state_ = state_commanded;
}

}  // namespace aic_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(aic_controller::Controller,
                       controller_interface::ControllerInterface)
