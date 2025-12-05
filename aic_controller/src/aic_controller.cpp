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
  msg = aic_controller::MotionUpdate();
}

// called from RT control loop
void reset_joint_motion_update_msg(aic_controller::JointMotionUpdate& msg) {
  msg = aic_controller::JointMotionUpdate();
}

}  // namespace

namespace aic_controller {

Controller::Controller()
    : num_joints_(0),
      has_position_state_interface_(false),
      has_velocity_state_interface_(false),
      has_acceleration_state_interface_(false),
      time_to_target_seconds_(0.0) {}

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

  // Initialize size and value of joint references and states
  reference_joints_ = JointTrajectoryPoint();
  reference_joints_.value().positions.assign(num_joints_, 0.0);
  reference_joints_.value().velocities.assign(num_joints_, 0.0);
  reference_joints_.value().accelerations.assign(num_joints_, 0.0);
  last_commanded_joints_ = reference_joints_.value();
  joint_state_ = reference_joints_.value();

  cartesian_impedance_action_ =
      std::make_unique<CartesianImpedanceAction>(num_joints_);

  if (!cartesian_impedance_action_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unable to create CartesianImpedanceAction");
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Controller::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;

  std::vector<std::string> command_interfaces_config_names;
  for (const auto& interface : params_.command_interfaces) {
    if (control_mode_ == ControlMode::Admittance &&
        interface == hardware_interface::HW_IF_POSITION) {
      // Only initialize position interfaces in admittance control mode
      for (const auto& joint : command_joint_names_) {
        auto full_name = joint + "/" + interface;
        command_interfaces_config_names.push_back(full_name);
      }
    } else if (control_mode_ == ControlMode::Impedance &&
               interface == hardware_interface::HW_IF_EFFORT) {
      // Only initialize effort interfaces in impedance control mode
      for (const auto& joint : command_joint_names_) {
        auto full_name = joint + "/" + interface;
        command_interfaces_config_names.push_back(full_name);
      }
    }
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          command_interfaces_config_names};
}

controller_interface::InterfaceConfiguration
Controller::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  std::vector<std::string> state_interfaces_config_names;

  for (const auto& interface : params_.state_interfaces) {
    for (const auto& joint : params_.joints) {
      auto full_name = joint + "/" + interface;
      state_interfaces_config_names.push_back(full_name);
    }
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          state_interfaces_config_names};
}

controller_interface::CallbackReturn Controller::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
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

  bool has_position_command_interface = contains_interface_type(
      params_.command_interfaces, hardware_interface::HW_IF_POSITION);
  bool has_effort_command_interface = contains_interface_type(
      params_.command_interfaces, hardware_interface::HW_IF_EFFORT);

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

  // validate control_mode
  if (control_mode_ == ControlMode::Admittance &&
      !has_position_command_interface) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Control mode set to 'admittance' but no position command "
                 "interface set");
    return controller_interface::CallbackReturn::FAILURE;
  } else if (control_mode_ == ControlMode::Impedance &&
             !has_effort_command_interface) {
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
    interpolation_mode_ = InterpolationMode::Linear;
    RCLCPP_INFO(get_node()->get_logger(), "Interpolation mode set to Linear");

  } else {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Unsupported interpolation mode. Please use 'linear' interpolation");

    return controller_interface::CallbackReturn::FAILURE;
  }

  // Validate target type
  if (params_.target_type == "cartesian") {
    RCLCPP_INFO(get_node()->get_logger(), "Target type set to CARTESIAN.");
    target_type_ = TargetType::Cartesian;
  } else if (params_.target_type == "joint") {
    RCLCPP_INFO(get_node()->get_logger(), "Target type set to JOINT.");
    target_type_ = TargetType::Joint;
  } else {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Unsupported target type. Please use either 'cartesian' or 'joints'");
    return controller_interface::CallbackReturn::FAILURE;
  }

  motion_update_sub_ = this->get_node()->create_subscription<MotionUpdate>(
      "~/motion_update", rclcpp::SystemDefaultsQoS(),
      [this](const MotionUpdate::SharedPtr msg) {
        if (target_type_ == TargetType::Joint) {
          RCLCPP_INFO_THROTTLE(get_node()->get_logger(),
                               *get_node()->get_clock(), 1000,
                               "Current target_type set to Joint, only "
                               "accepting JointMotionUpdate messages");
          return;
        }

        motion_update_rt_.set(*msg);
      });

  joint_motion_update_sub_ =
      this->get_node()->create_subscription<JointMotionUpdate>(
          "~/joint_motion_update", rclcpp::SystemDefaultsQoS(),
          [this](const JointMotionUpdate::SharedPtr msg) {
            if (target_type_ == TargetType::Cartesian) {
              RCLCPP_INFO_THROTTLE(get_node()->get_logger(),
                                   *get_node()->get_clock(), 1000,
                                   "Current target_type set to Cartesian, only "
                                   "accepting MotionUpdate messages");
              return;
            }

            joint_motion_update_rt_.set(*msg);
          });

  if (control_mode_ == ControlMode::Impedance) {
    if (!cartesian_impedance_action_) {
      return controller_interface::CallbackReturn::ERROR;
    }

    if (!cartesian_impedance_action_->Configure(
            get_node(), this->get_robot_description())) {
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Controller::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (control_mode_ == ControlMode::Impedance) {
    if (!cartesian_impedance_action_) {
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // read and initialize current joint states
  read_state_from_hardware(joint_state_);
  for (const auto& val : joint_state_.positions) {
    if (std::isnan(val)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to read joint positions from the hardware.\n");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  if (!reference_joints_.has_value()) {
    reference_joints_ = joint_state_;
  }
  if (!target_joints_.has_value()) {
    target_joints_ = joint_state_;
  }
  last_commanded_joints_ = joint_state_;

  // todo(johntgz) Set motion_update_ to the current sensed state
  reset_motion_update_msg(motion_update_);
  motion_update_rt_.try_set(motion_update_);

  reset_joint_motion_update_msg(joint_motion_update_);
  joint_motion_update_.trajectory_generation_mode.mode =
      TrajectoryGenerationMode::MODE_POSITION;
  joint_motion_update_.target_state = joint_state_;
  joint_motion_update_rt_.try_set(joint_motion_update_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Controller::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (control_mode_ == ControlMode::Impedance) {
    if (!cartesian_impedance_action_) {
      return controller_interface::CallbackReturn::ERROR;
    }

    release_interfaces();
  }

  reset_motion_update_msg(motion_update_);
  motion_update_rt_.try_set(motion_update_);

  reset_joint_motion_update_msg(joint_motion_update_);
  joint_motion_update_rt_.try_set(joint_motion_update_);

  return controller_interface::CallbackReturn::SUCCESS;
}

bool Controller::update_cartesian_reference() {
  RCLCPP_ERROR(get_node()->get_logger(),
               "update_cartesian_reference() is unimplemented");
  return false;
}

bool Controller::update_joint_reference() {
  if (!reference_joints_.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unset reference_joints_ in update_joint_reference()");
    return false;
  }

  if (!target_joints_.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unset target_joints_ in update_joint_reference()");
    return false;
  }

  // update target to ensure it is within pre-defined limits
  switch (joint_motion_update_.trajectory_generation_mode.mode) {
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
      return false;

      break;
    case TrajectoryGenerationMode::MODE_POSITION_AND_VELOCITY:
      // UNIMPLEMENTED
      // Clamp pose and twist to limit
      RCLCPP_ERROR(get_node()->get_logger(),
                   "MODE_POSITION_AND_VELOCITY trajectory generation mode is "
                   "unimplemented. Please use MODE_POSITION.");
      return false;

      break;
    default:
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Unsupported trajectory generation mode. Please set to "
                   "either MODE_POSITION, MODE_VELOCITY or "
                   "MODE_POSITION_AND_VELOCITY");
      return false;
  }

  time_to_target_seconds_ = joint_motion_update_.time_to_target_seconds;

  auto new_reference = *reference_joints_;
  switch (interpolation_mode_) {
    case InterpolationMode::Linear:
      // UNIMPLEMENTED
      if (!update_reference_joints_linear_interpolation(
              reference_joints_.value(), target_joints_.value(),
              new_reference)) {
        return false;
      }

      break;
    default:
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Invalid interpolation mode. Only supported interpolation "
                   "mode is 'linear'");
      return false;
      break;
  }

  reference_joints_ = new_reference;

  return true;
}

controller_interface::return_type Controller::update_and_write_commands(
    const ControlMode& control_mode, const TargetType& target_type) {
  JointTrajectoryPoint reference;
  if (control_mode == ControlMode::Impedance) {
    // UNIMPLEMENTED
    // Interpolate impedance parameters
    //    UpdateImpedance(target_type)
    // Interpolate feed-forward wrench
    //    UpdateFeedforwardWrench(target_type)

    if (target_type == TargetType::Cartesian) {
      // UNIMPLEMENTED
      // Compute joint torques
    } else if (target_type == TargetType::Joint) {
      // UNIMPLEMENTED
      // Compute joint torques
    }
    // UNIMPLEMENTED
    // Write the control torque to hardware interfaces

    RCLCPP_ERROR(get_node()->get_logger(),
                 "Impedance control is unimplemented.");

    return controller_interface::return_type::ERROR;
  } else if (control_mode == ControlMode::Admittance) {
    if (target_type == TargetType::Cartesian) {
      // UNIMPLEMENTED
      // Compute joint references given cartesian target using IK

      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Cartesian targets for admittance controller is unimplemented.");

      return controller_interface::return_type::ERROR;
    } else if (target_type == TargetType::Joint) {
      // Simply forward the joint reference to the admittance controller
      reference = reference_joints_.value();
    }

  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Invalid control mode defined. Please set control_mode to "
                 "either 'admittance' or 'impedance'");

    return controller_interface::return_type::ERROR;
  }

  write_state_to_hardware(reference);

  return controller_interface::return_type::OK;
}

controller_interface::return_type Controller::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Read and update current states from sensors
  if (!sense()) {
    return controller_interface::return_type::ERROR;
  }

  // Update reference by limiting them and interpolating their values
  if (target_type_ == TargetType::Cartesian) {
    if (!update_cartesian_reference()) {
      return controller_interface::return_type::ERROR;
    }
  } else if (target_type_ == TargetType::Joint) {
    if (!update_joint_reference()) {
      return controller_interface::return_type::ERROR;
    }
  }

  update_and_write_commands(control_mode_, target_type_);

  return controller_interface::return_type::OK;
}

bool Controller::sense() {
  read_state_from_hardware(joint_state_);

  // read user commands
  if (target_type_ == TargetType::Cartesian) {
    auto command_op = motion_update_rt_.try_get();
    if (command_op.has_value()) {
      motion_update_ = command_op.value();
    }

  } else if (target_type_ == TargetType::Joint) {
    auto command_op = joint_motion_update_rt_.try_get();
    if (command_op.has_value()) {
      joint_motion_update_ = command_op.value();
      target_joints_ = joint_motion_update_.target_state;
    }
  }

  if (control_mode_ == ControlMode::Impedance) {
    if (target_type_ == TargetType::Cartesian) {
      // UNIMPLEMENTED
      cartesian_impedance_action_->Update(joint_state_);
    } else if (target_type_ == TargetType::Joint) {
      // UNIMPLEMENTED
      // update joint impedance controller with current joint state
    }
  }

  return true;
}

bool Controller::update_reference_joints_linear_interpolation(
    const JointTrajectoryPoint& reference_state,
    const JointTrajectoryPoint& target_state,
    JointTrajectoryPoint& new_reference) {
  new_reference = target_state;
  return true;
}

void Controller::read_state_from_hardware(JointTrajectoryPoint& state_current) {
  // Set state_current to last commanded state if any of the hardware interface
  // values are NaN
  bool nan_position = false;
  bool nan_velocity = false;
  bool nan_acceleration = false;

  std::size_t pos_ind = 0;
  std::size_t vel_ind = pos_ind + has_velocity_state_interface_;
  std::size_t acc_ind = vel_ind + has_acceleration_state_interface_;

  for (std::size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind) {
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
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Read NaN value from position state interface, setting "
                 "current position to last_commanded_joints_");
    state_current.positions = last_commanded_joints_.positions;
  }
  if (nan_velocity) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Read NaN value from velocity state interface, setting "
                 "current velocity to last_commanded_joints_");
    state_current.velocities = last_commanded_joints_.velocities;
  }
  if (nan_acceleration) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Read NaN value from acceleration state interface, setting "
                 "current acceleration to last_commanded_joints_");
    state_current.accelerations = last_commanded_joints_.accelerations;
  }
}

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

  last_commanded_joints_ = state_commanded;
}

}  // namespace aic_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(aic_controller::Controller,
                       controller_interface::ControllerInterface)
