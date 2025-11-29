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

namespace {  // utility

// called from RT control loop
void reset_motion_update_msg(aic_controller::MotionUpdate& msg) {
  // todo(johntgz) reset values?
  msg = aic_controller::MotionUpdate();
}

// called from RT control loop
void reset_joint_motion_update_msg(aic_controller::JointMotionUpdate& msg) {
  // todo(johntgz) reset to a nan value?
  msg = aic_controller::JointMotionUpdate();
}

}  // namespace

namespace aic_controller {

controller_interface::CallbackReturn AICController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
    // impedance_ =
    //     std::make_unique<aic_controller::ImpedanceRule>(param_listener_);

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // number of joints in controllers is fixed after initialization
  num_joints_ = params_.joints.size();

  // allocate dynamic memory
  reference_joints_.positions.assign(num_joints_, 0.0);
  reference_joints_.velocities.assign(num_joints_, 0.0);
  reference_joints_.accelerations.assign(num_joints_, 0.0);
  last_commanded_joints_ = reference_joints_;
  joint_state_ = reference_joints_;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AICController::command_interface_configuration() const {
  RCLCPP_INFO(get_node()->get_logger(), "command_interface_configuration().");

  controller_interface::InterfaceConfiguration command_interfaces_config;

  std::vector<std::string> command_interfaces_config_names;
  for (const auto& interface : params_.command_interfaces) {
    for (const auto& joint : command_joint_names_) {
      auto full_name = joint + "/" + interface;
      command_interfaces_config_names.push_back(full_name);
    }
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          command_interfaces_config_names};
}

controller_interface::InterfaceConfiguration
AICController::state_interface_configuration() const {
  RCLCPP_INFO(get_node()->get_logger(), "state_interface_configuration().");
  controller_interface::InterfaceConfiguration state_interfaces_config;
  std::vector<std::string> state_interfaces_config_names;

  for (const auto& interface : params_.state_interfaces) {
    for (const auto& joint : params_.joints) {
      auto full_name = joint + "/" + interface;
      state_interfaces_config_names.push_back(full_name);
    }
  }

  // TODO(johntgz) disable this based on control_mode
  if (control_mode_ == ControlMode::IMPEDANCE) {
    auto ft_interfaces = force_torque_sensor_->get_state_interface_names();
    state_interfaces_config_names.insert(state_interfaces_config_names.end(),
                                         ft_interfaces.begin(),
                                         ft_interfaces.end());
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          state_interfaces_config_names};
}

controller_interface::CallbackReturn AICController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(this->get_node()->get_logger(), "configuring...");

  // Set and validate commanded joint names
  command_joint_names_ = params_.command_joints;
  if (command_joint_names_.empty()) {
    command_joint_names_ = params_.joints;
    RCLCPP_INFO(get_node()->get_logger(),
                "No specific joint names are used for command interfaces. "
                "Using 'joints' parameter.");
  } else if (command_joint_names_.size() != num_joints_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "'command_joints' parameter needs to have the same size as "
                 "'joints' parameter.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // print and validate interface types
  for (const auto& tmp : params_.state_interfaces) {
    RCLCPP_INFO(get_node()->get_logger(), "%s",
                ("state int types are: " + tmp + "\n").c_str());
  }
  for (const auto& tmp : params_.command_interfaces) {
    RCLCPP_INFO(get_node()->get_logger(), "%s",
                ("command int types are: " + tmp + "\n").c_str());
  }

  // Check if only allowed interface types are used
  auto contains_interface_type =
      [](const std::vector<std::string>& interface_type_list,
         const std::string& interface_type) {
        return std::find(interface_type_list.begin(), interface_type_list.end(),
                         interface_type) != interface_type_list.end();
      };

  has_position_command_interface_ = contains_interface_type(
      params_.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_effort_command_interface_ = contains_interface_type(
      params_.command_interfaces, hardware_interface::HW_IF_EFFORT);

  if (params_.control_mode == "impedance") {
    RCLCPP_INFO(get_node()->get_logger(), "Control mode set to impedance");
    control_mode_ = ControlMode::IMPEDANCE;
  } else if (params_.control_mode == "admittance") {
    RCLCPP_INFO(get_node()->get_logger(), "Control mode set to admittance");
    control_mode_ = ControlMode::ADMITTANCE;
  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unsupported control mode. Please set control_mode to either "
                 "'admittance' or 'impedance'");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // validate control_mode
  if (control_mode_ == ControlMode::ADMITTANCE &&
      !has_position_command_interface_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Control mode set to 'admittance' but no position command "
                 "interface set");
    return controller_interface::CallbackReturn::FAILURE;
  } else if (control_mode_ == ControlMode::IMPEDANCE &&
             !has_effort_command_interface_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Control mode set to 'impedance' but no torque command "
                 "interface set");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used
  for (const auto& interface : params_.state_interfaces) {
    auto it = std::find(allowed_state_interface_types_.begin(),
                        allowed_state_interface_types_.end(), interface);
    if (it == allowed_state_interface_types_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "State interface type '%s' not allowed!", interface.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  has_position_state_interface_ = contains_interface_type(
      params_.state_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_state_interface_ = contains_interface_type(
      params_.state_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_state_interface_ = contains_interface_type(
      params_.state_interfaces, hardware_interface::HW_IF_ACCELERATION);

  if (!has_position_state_interface_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Position state interface is required.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Validate interpolation mode
  if (params_.interpolation_mode == "linear") {
    RCLCPP_INFO(get_node()->get_logger(), "Interpolation mode set to LINEAR");
    interpolation_mode_ = InterpolationMode::LINEAR;
  } else if (params_.interpolation_mode == "reflexxes") {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unimplemented interpolation mode 'reflexxes'. Please use "
                 "'linear' interpolation");
    return controller_interface::CallbackReturn::FAILURE;
  } else if (params_.interpolation_mode == "minimal_splines") {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unimplemented interpolation mode 'minimal_splines'. Please "
                 "use 'linear' interpolation");
    return controller_interface::CallbackReturn::FAILURE;
  } else {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Unsupported interpolation mode. Please use 'linear' interpolation");
    return controller_interface::CallbackReturn::FAILURE;
  }

  motion_update_sub_ = this->get_node()->create_subscription<MotionUpdate>(
      "~/motion_update", rclcpp::SystemDefaultsQoS(),
      [this](const MotionUpdate::SharedPtr msg) {
        RCLCPP_INFO(
            get_node()->get_logger(),
            "Received MotionUpdate command with: \n Trajectory Generation "
            "Mode "
            ": %d \n Pose : (% .2f, % .2f, % .2f) \n Feedforward Wrench : (% "
            ".2f, % .2f, % .2f) \n time_to_target : %.2f s \n ",
            msg->trajectory_generation_mode.mode, msg->pose.position.x,
            msg->pose.position.y, msg->pose.position.z,
            msg->feedforward_wrench_at_tip.force.x,
            msg->feedforward_wrench_at_tip.force.y,
            msg->feedforward_wrench_at_tip.force.z,
            msg->time_to_target_seconds);

        motion_update_command_.set(*msg);

        target_type_ = TargetType::CARTESIAN;
      });

  joint_motion_update_sub_ =
      this->get_node()->create_subscription<JointMotionUpdate>(
          "~/joint_motion_update", rclcpp::SystemDefaultsQoS(),
          [this](const JointMotionUpdate::SharedPtr msg) {
            RCLCPP_INFO(
                get_node()->get_logger(),
                "Received JointMotionUpdate command with: \n Trajectory "
                "Generation Mode "
                ": %d \n time_to_target : %.2f s \n ",
                msg->trajectory_generation_mode.mode,
                msg->time_to_target_seconds);

            joint_motion_update_command_.set(*msg);

            target_type_ = TargetType::JOINT;
          });

  if (control_mode_ == ControlMode::IMPEDANCE) {
    force_torque_sensor_ =
        std::make_unique<semantic_components::ForceTorqueSensor>(
            params_.ft_sensor.name);

    // if (
    //   impedance_->configure(get_node(), num_joints_,
    //   this->get_robot_description()) ==
    //   controller_interface::return_type::ERROR)
    // {
    //   return controller_interface::CallbackReturn::ERROR;
    // }
  }

  RCLCPP_INFO(this->get_node()->get_logger(), "configure successful");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AICController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(this->get_node()->get_logger(), "activating...");

  if (control_mode_ == ControlMode::IMPEDANCE) {
    // if (!impedance_)
    // {
    //   return controller_interface::CallbackReturn::ERROR;
    // }

    // update parameters if any have changed
    // impedance_->apply_parameters_update();

    force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);
    read_force_torque_sensor_states(ft_values_);

  } else if (control_mode_ == ControlMode::ADMITTANCE) {
  }

  // initialize states
  read_joint_states(joint_state_);

  for (auto val : joint_state_.positions) {
    if (std::isnan(val)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to read joint positions from the hardware.\n");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  if (!reference_joints_.has_value()) {
    reference_joints_ = joint_state_;
  }
  if (!target_joint_state_.has_value()) {
    target_joint_state_ = joint_state_;
  }
  last_commanded_joints_ = joint_state_;

  reset_motion_update_msg(motion_update_msg_);
  motion_update_command_.try_set(motion_update_msg_);

  reset_joint_motion_update_msg(joint_motion_update_msg_);
  joint_motion_update_command_.try_set(joint_motion_update_msg_);

  RCLCPP_INFO(this->get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AICController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (control_mode_ == ControlMode::IMPEDANCE) {
    // if (!impedance_)
    // {
    //   return controller_interface::CallbackReturn::ERROR;
    // }

    // release force torque sensor interface
    force_torque_sensor_->release_interfaces();

    release_interfaces();
    // impedance_->reset(num_joints_);

  } else if (control_mode_ == ControlMode::ADMITTANCE) {
    // todo(johntgz) reset to prevent stale references
  }

  reset_motion_update_msg(motion_update_msg_);
  motion_update_command_.try_set(motion_update_msg_);

  reset_joint_motion_update_msg(joint_motion_update_msg_);
  joint_motion_update_command_.try_set(joint_motion_update_msg_);

  RCLCPP_INFO(this->get_node()->get_logger(), "deactivate successful");

  return controller_interface::CallbackReturn::SUCCESS;
}

bool AICController::update_reference_cartesian() {
  RCLCPP_ERROR(get_node()->get_logger(),
               "update_reference_cartesian() is unimplemented.");
  return false;
}

bool AICController::update_reference_joints() {
  if (!reference_joints_.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unset reference_joints_ in update_reference_joints()");
    return false;
  }

  if (!target_joint_state_.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unset target_joint_state_ in update_reference_joints()");
    return false;
  }

  // todo(@johntgz) add interpolation logic
  switch (joint_motion_update_msg_.trajectory_generation_mode.mode) {
    case TrajectoryGenerationMode::MODE_POSITION:
      break;
    case TrajectoryGenerationMode::MODE_VELOCITY:
      RCLCPP_ERROR(get_node()->get_logger(),
                   "MODE_VELOCITY trajectory generation mode is unimplemented. "
                   "Please use MODE_POSITION.");
      return false;
      break;
    case TrajectoryGenerationMode::MODE_POSITION_AND_VELOCITY:
      RCLCPP_ERROR(get_node()->get_logger(),
                   "MODE_POSITION_AND_VELOCITY trajectory generation mode is "
                   "unimplemented. Please use MODE_POSITION.");
      return false;
      break;
    case TrajectoryGenerationMode::MODE_UNSPECIFIED:

      break;
    default:
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Unsupported trajectory generation mode. Please set to "
                   "either MODE_POSITION, MODE_VELOCITY or "
                   "MODE_POSITION_AND_VELOCITY");
      return false;
  }

  time_to_target_seconds_ = joint_motion_update_msg_.time_to_target_seconds;

  auto new_reference_joints_ = *reference_joints_;
  switch (interpolation_mode_) {
    case InterpolationMode::LINEAR:
      new_reference_joints_ =
          update_reference_joints_linear_interpolation(*target_joint_state_);
      break;
    case InterpolationMode::REFLEXXES:
      RCLCPP_WARN(
          get_node()->get_logger(),
          "REFLEXXES interpolation mode is unimplemented. Please use LINEAR");
      return false;
      break;
    case InterpolationMode::MINIMAL_SPLINES:
      RCLCPP_WARN(get_node()->get_logger(),
                  "MINIMAL_SPLINES interpolation mode is unimplemented. Please "
                  "use LINEAR");
      return false;
      break;
    case InterpolationMode::UNSPECIFIED_INTERPOLATION_MODE:
      RCLCPP_WARN(get_node()->get_logger(),
                  "Unspecified interpolation mode. Please use LINEAR");
      return false;
      break;
    default:
      RCLCPP_WARN(get_node()->get_logger(),
                  "Unknown interpolation mode. Please use LINEAR");
      return false;
      break;
  }

  reference_joints_ = new_reference_joints_;

  return true;
}

controller_interface::return_type
AICController::update_and_write_commands_cartesian() {
  if (!update_reference_cartesian()) {
    return controller_interface::return_type::ERROR;
  }

  if (control_mode_ == ControlMode::IMPEDANCE) {
  } else if (control_mode_ == ControlMode::ADMITTANCE) {
  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unsupported control_mode. Please set to either 'admittance' "
                 "or 'impedance'");
    return controller_interface::return_type::ERROR;
  }

  RCLCPP_ERROR(
      get_node()->get_logger(),
      "Cartesian targets via MotionUpdate commands are unimplemented. Please "
      "use joint targets by publishgin JointMotionCommands instead.");

  return controller_interface::return_type::ERROR;
}

controller_interface::return_type
AICController::update_and_write_commands_joints() {
  if (!update_reference_joints()) {
    return controller_interface::return_type::ERROR;
  }

  if (control_mode_ == ControlMode::IMPEDANCE) {
    read_force_torque_sensor_states(ft_values_);
  } else if (control_mode_ == ControlMode::ADMITTANCE) {
    write_reference_to_admittance_controller(*reference_joints_);
  } else {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Unsupported control_mode. Supported control modes are 'admittance' "
        "or 'impedance'");
    return controller_interface::return_type::ERROR;
  }

  // todo(johntgz) Publish controller state

  return controller_interface::return_type::OK;
}

controller_interface::return_type AICController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  sense();

  if (target_type_ == TargetType::CARTESIAN) {
    return update_and_write_commands_cartesian();
  } else if (target_type_ == TargetType::JOINT) {
    return update_and_write_commands_joints();
  }

  return controller_interface::return_type::OK;
}

void AICController::read_joint_states(JointTrajectoryPoint& state_current) {
  // if any interface has nan values, assume state_current is the last command
  // state
  bool nan_position = false;
  bool nan_velocity = false;
  bool nan_acceleration = false;

  size_t pos_ind = 0;
  size_t vel_ind = pos_ind + has_velocity_state_interface_;
  size_t acc_ind = vel_ind + has_acceleration_state_interface_;

  for (size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind) {
    if (has_position_state_interface_) {
      const auto state_current_position_op =
          state_interfaces_[pos_ind * num_joints_ + joint_ind].get_optional();
      nan_position |= !state_current_position_op.has_value() ||
                      std::isnan(state_current_position_op.value());
      if (state_current_position_op.has_value()) {
        state_current.positions[joint_ind] = state_current_position_op.value();
      }
    }
    if (has_velocity_state_interface_) {
      auto state_current_velocity_op =
          state_interfaces_[vel_ind * num_joints_ + joint_ind].get_optional();
      nan_velocity |= !state_current_velocity_op.has_value() ||
                      std::isnan(state_current_velocity_op.value());

      if (state_current_velocity_op.has_value()) {
        state_current.velocities[joint_ind] = state_current_velocity_op.value();
      }
    }
    if (has_acceleration_state_interface_) {
      auto state_current_acceleration_op =
          state_interfaces_[acc_ind * num_joints_ + joint_ind].get_optional();
      nan_acceleration |= !state_current_acceleration_op.has_value() ||
                          std::isnan(state_current_acceleration_op.value());
      if (state_current_acceleration_op.has_value()) {
        state_current.accelerations[joint_ind] =
            state_current_acceleration_op.value();
      }
    }
  }

  if (nan_position) {
    state_current.positions = last_commanded_joints_.positions;
  }
  if (nan_velocity) {
    state_current.velocities = last_commanded_joints_.velocities;
  }
  if (nan_acceleration) {
    state_current.accelerations = last_commanded_joints_.accelerations;
  }
}

void AICController::read_force_torque_sensor_states(
    geometry_msgs::msg::Wrench& ft_values) {
  // if any ft_values are nan, assume values are zero
  force_torque_sensor_->get_values_as_message(ft_values);
  if (std::isnan(ft_values.force.x) || std::isnan(ft_values.force.y) ||
      std::isnan(ft_values.force.z) || std::isnan(ft_values.torque.x) ||
      std::isnan(ft_values.torque.y) || std::isnan(ft_values.torque.z)) {
    ft_values = geometry_msgs::msg::Wrench();
  }
}

void AICController::write_reference_to_admittance_controller(
    const JointTrajectoryPoint& state_commanded) {
  // if any interface has nan values, assume state_commanded is the last
  // command state
  size_t pos_ind = 0;

  for (size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind) {
    bool success = true;
    if (has_position_command_interface_) {
      success &=
          command_interfaces_[pos_ind * num_joints_ + joint_ind].set_value(
              state_commanded.positions[joint_ind]);
    }
    if (!success) {
      RCLCPP_WARN(this->get_node()->get_logger(),
                  "Error while setting command for joint %zu.", joint_ind);
    }
  }

  last_commanded_joints_ = state_commanded;
}

controller_interface::return_type AICController::sense() {
  // Read current joint states
  read_joint_states(joint_state_);

  // read user commands
  if (target_type_ == TargetType::CARTESIAN) {
    auto command_op = motion_update_command_.try_get();
    if (command_op.has_value()) {
      motion_update_msg_ = command_op.value();
    }

  } else if (target_type_ == TargetType::JOINT) {
    auto command_op = joint_motion_update_command_.try_get();
    if (command_op.has_value()) {
      joint_motion_update_msg_ = command_op.value();
      target_joint_state_ = joint_motion_update_msg_.target_state;
    }
  }

  return controller_interface::return_type::OK;
}

JointTrajectoryPoint
AICController::update_reference_joints_linear_interpolation(
    const JointTrajectoryPoint& msg) {
  // todo(johntgz) implement this
  return msg;
}

}  // namespace aic_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(aic_controller::AICController,
                       controller_interface::ControllerInterface)
