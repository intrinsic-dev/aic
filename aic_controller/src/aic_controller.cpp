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
      command_joint_names_({}),
      target_type_(TargetType::Invalid),
      control_mode_(ControlMode::Invalid),
      interpolation_mode_(InterpolationMode::Invalid),
      has_position_state_interface_(false),
      has_velocity_state_interface_(false),
      motion_update_sub_(nullptr),
      last_commanded_state_(std::nullopt),
      current_state_(std::nullopt),
      time_to_target_seconds_(0.0) {
  // Do nothing.
}

//==============================================================================
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

//==============================================================================
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

  // Initialize size and value of joint references and states
  // TODO(Yadunund): why are next_command_ and current_state_ optional if they
  // always get initialized here?
  // TODO(Yadunund): why are last_commanded_state_ and current_state_ not
  // optional and instead being initialized directly? current_state_ should be
  // initialized only in on_activate().

  return controller_interface::CallbackReturn::SUCCESS;
}

//==============================================================================
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

  if (!has_position_state_interface_ || !has_velocity_state_interface_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Both position and velocity state interfaces are required.");
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
  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unsupported target type. Please use 'cartesian'");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Reliable QoS subscriptions for motion updates.
  rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  motion_update_sub_ = this->get_node()->create_subscription<MotionUpdate>(
      "~/motion_update", reliable_qos,
      [this](const MotionUpdate::SharedPtr msg) {
        // todo(johntgz) Is RealtimeThreadSafeBox necessary? Can we avoid a copy
        // here?
        motion_update_rt_.set(*msg);
      });

  return controller_interface::CallbackReturn::SUCCESS;
}

//==============================================================================
controller_interface::CallbackReturn Controller::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // read and initialize current joint states
  if (!current_state_.has_value()) {
    current_state_ = JointTrajectoryPoint();
    current_state_.value().positions.assign(num_joints_, 0.0);
    current_state_.value().velocities.assign(num_joints_, 0.0);
  }
  read_state_from_hardware(current_state_.value());
  for (const auto& val : current_state_.value().positions) {
    if (std::isnan(val)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to read joint positions from the hardware.\n");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Set the current state as the default command
  next_command_ = current_state_.value();
  // todo(johntgz) Initialize target_state_ to current tool state from FK on
  // current joint state

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
bool Controller::update_reference() {
  // update target to ensure it is within pre-defined limits
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

  time_to_target_seconds_ = motion_update_.time_to_target_seconds;

  auto new_reference = next_command_;
  switch (interpolation_mode_) {
    case InterpolationMode::Linear:
      // UNIMPLEMENTED
      // Apply linear interpolation to the target_state_ to obtain a new
      // reference. Linear interpolation should support MODE_POSITION,
      // MODE_VELOCITY and MODE_POSITION_AND_VELOCITY
      new_reference = target_state_;

      break;
    default:
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Invalid interpolation mode. Only supported interpolation "
                   "mode is 'linear'");
      return false;
      break;
  }

  next_command_ = new_reference;

  return true;
}

//==============================================================================
controller_interface::return_type Controller::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Read and update current states from sensors
  read_state_from_hardware(current_state_.value());

  // read user commands
  auto command_op = motion_update_rt_.try_get();
  if (command_op.has_value()) {
    motion_update_ = command_op.value();
    // UNIMPLEMENTED: Update target
    // todo(johntgz)
    // target_state_ = CartState(motion_update_.pose, motion_update_.velocity,
    //                           motion_update_.acceleration);
  }

  if (control_mode_ == ControlMode::Impedance) {
    // UNIMPLEMENTED
    // update joint impedance controller with current joint state
  }

  // Update reference by limiting them and interpolating their values
  if (target_type_ == TargetType::Cartesian) {
    if (!update_reference()) {
      return controller_interface::return_type::ERROR;
    }
  } else {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Other target types are unimplemented. Please use 'cartesian'");

    return controller_interface::return_type::ERROR;
  }

  JointTrajectoryPoint reference;
  if (control_mode_ == ControlMode::Impedance) {
    // UNIMPLEMENTED
    // Interpolate impedance parameters
    //    UpdateImpedance(target_type_)
    // Interpolate feed-forward wrench
    //    UpdateFeedforwardWrench(target_type_)

    if (target_type_ == TargetType::Cartesian) {
      // UNIMPLEMENTED
      // Compute joint torques
    } else {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Other target types are unimplemented. Please use 'cartesian'");

      return controller_interface::return_type::ERROR;
    }
    // UNIMPLEMENTED
    // Write the control torque to hardware interfaces

    RCLCPP_ERROR(get_node()->get_logger(),
                 "Impedance control is unimplemented.");

    return controller_interface::return_type::ERROR;
  } else if (control_mode_ == ControlMode::Admittance) {
    if (target_type_ == TargetType::Cartesian) {
      // Simply forward the joint reference to the admittance controller
      reference = next_command_;
    } else {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Other target types are unimplemented. Please use 'cartesian'");

      return controller_interface::return_type::ERROR;
    }
  }

  write_state_to_hardware(reference);

  return controller_interface::return_type::OK;
}

//==============================================================================
void Controller::read_state_from_hardware(JointTrajectoryPoint& state_current) {
  // Set state_current to last commanded state if any of the hardware interface
  // values are NaN
  bool nan_position = false;
  bool nan_velocity = false;

  std::size_t pos_ind = 0;
  std::size_t vel_ind = pos_ind + has_velocity_state_interface_;

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
