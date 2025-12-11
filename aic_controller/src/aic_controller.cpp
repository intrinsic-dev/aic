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

}  // namespace

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
      remaining_time_to_target_seconds_(0.0),
      kinematics_loader_(nullptr),
      kinematics_(nullptr) {
  // Do nothing.
}

//==============================================================================
controller_interface::InterfaceConfiguration
Controller::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  std::vector<std::string> command_interfaces_config_names;

  const std::string controller_prefix =
      control_mode_ == ControlMode::Admittance
          ? params_.admittance_controller_namespace + "/"
          : "";
  const std::string interface = control_mode_ == ControlMode::Admittance
                                    ? hardware_interface::HW_IF_POSITION
                                    : hardware_interface::HW_IF_EFFORT;

  for (const auto& joint : params_.joints) {
    command_interfaces_config_names.push_back(controller_prefix + joint + "/" +
                                              interface);
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
    state_interfaces_config_names.push_back(joint + "/" +
                                            hardware_interface::HW_IF_POSITION);
    state_interfaces_config_names.push_back(joint + "/" +
                                            hardware_interface::HW_IF_VELOCITY);
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

  // Validate control frequency
  if (params_.control_frequency <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Control frqeuency needs to be set to a positive number, "
                 "current set as %f Hz",
                 params_.control_frequency);
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Reliable QoS subscriptions for motion updates.
  rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  motion_update_sub_ = this->get_node()->create_subscription<MotionUpdate>(
      "~/motion_update", reliable_qos,
      [this](const MotionUpdate::SharedPtr msg) {
        if (get_node()->get_current_state().id() !=
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
                               *get_node()->get_clock(), 1000,
                               "Controller is not in ACTIVE lifecycle state, "
                               "ignoring MotionUpdate message.");

          return;
        }

        motion_update_rt_.set(*msg);
        motion_update_received_ = true;
      });

  // Load the kinematics plugin
  if (!params_.kinematics.plugin_name.empty()) {
    try {
      // Reset the interface first to avoid a segfault
      if (kinematics_loader_) {
        kinematics_.reset();
      }
      kinematics_loader_ = std::make_shared<
          pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
          params_.kinematics.plugin_package,
          "kinematics_interface::KinematicsInterface");
      kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
          kinematics_loader_->createUnmanagedInstance(
              params_.kinematics.plugin_name));

      if (!kinematics_->initialize(
              this->get_robot_description(),
              this->get_node()->get_node_parameters_interface(),
              "kinematics")) {
        return controller_interface::CallbackReturn::ERROR;
      }
    } catch (pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Exception while loading the IK plugin '%s': '%s'",
                   params_.kinematics.plugin_name.c_str(), ex.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "An IK plugin name was not specified in the config file.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!cartesian_impedance_action_->Configure(get_node(),
                                              this->get_robot_description())) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // todo(johntgz)
  // remove cartesian test method and set cartesian limits using parameters
  cartesian_limits_ = CartesianLimits::GenerateTestParams();

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

  if (!kinematics_->calculate_link_transform(current_state_.positions,
                                             params_.tool_frame_id,
                                             current_tool_state_.pose)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unable to compute current cartesian state of tool frame");
    return controller_interface::CallbackReturn::ERROR;
  }
  last_tool_reference_ = current_tool_state_;

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

  kinematics_loader_.reset();
  kinematics_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

//==============================================================================
controller_interface::return_type Controller::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Read and update current states from sensors
  read_state_from_hardware(current_state_);
  // Use forward kinematics to update the current cartesian state of tool frame
  if (!kinematics_->calculate_link_transform(current_state_.positions,
                                             params_.tool_frame_id,
                                             current_tool_state_.pose)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unable to compute current cartesian state of tool frame");
    return controller_interface::return_type::ERROR;
  }

  // read user commands
  if (motion_update_received_) {
    auto command_op = motion_update_rt_.try_get();
    if (command_op.has_value()) {
      motion_update_ = command_op.value();
      target_state_ =
          CartesianState(motion_update_.pose, motion_update_.velocity);
    }
  }

  if (!target_state_.has_value()) {
    // If target_state_ has no value, return early as there is nothing
    // to write to the hardware interfaces.
    return controller_interface::return_type::OK;
  }

  CartesianState new_tool_reference = last_tool_reference_;

  // Clamp the target states to stay within limits
  if (ClampReferenceToLimits(cartesian_limits_,
                             motion_update_.trajectory_generation_mode.mode,
                             target_state_.value())) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                         1000,
                         "Limit violation: Target has been clamped to limits");
  }

  time_to_target_seconds_ = motion_update_.time_to_target_seconds;

  // UNIMPLEMENTED
  // Apply linear interpolation to the target_state_ to obtain a new
  // reference. Linear interpolation should support MODE_POSITION,
  // MODE_VELOCITY and MODE_POSITION_AND_VELOCITY
  if (!UpdateReferenceLinearInterpolation(
          last_tool_reference_, target_state_.value(),
          remaining_time_to_target_seconds_, params_.control_frequency,
          motion_update_.trajectory_generation_mode.mode, new_tool_reference)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Linear interpolation of target failed");
    return controller_interface::return_type::ERROR;
  }

  // Compute controls
  JointTrajectoryPoint new_joint_reference;
  if (control_mode_ == ControlMode::Impedance) {
    // UNIMPLEMENTED
    // Interpolate impedance parameters and feed-forward wrench
    // Compute control torques

    new_joint_reference = cartesian_impedance_action_->Compute(
        new_tool_reference, current_state_);

    RCLCPP_ERROR(get_node()->get_logger(),
                 "Impedance control is unimplemented.");

    return controller_interface::return_type::ERROR;
  } else if (control_mode_ == ControlMode::Admittance) {
    // todo(johntgz) the code below may cause problems for configurations close
    // to singularity

    // Perform IK to get joint targets from target_state_
    Eigen::Matrix<double, 7, 1> current_cartesian_frame,
        reference_cartesian_frame;
    current_cartesian_frame.head<3>() = current_tool_state_.pose.translation();
    current_cartesian_frame.tail<4>() =
        current_tool_state_.getPoseQuaternion().coeffs();
    reference_cartesian_frame.head<3>() = new_tool_reference.pose.translation();
    reference_cartesian_frame.tail<4>() =
        new_tool_reference.getPoseQuaternion().coeffs();

    // Calculate the cartesian delta between the current and the reference tool
    // frame
    Eigen::Matrix<double, 6, 1> cartesian_delta;
    kinematics_->calculate_frame_difference(
        current_cartesian_frame, reference_cartesian_frame,
        (1.0 / params_.control_frequency), cartesian_delta);
    std::vector<double> cartesian_delta_vec(cartesian_delta.begin(),
                                            cartesian_delta.end());

    // Calculate the joint delta and use it to get the reference joint
    // position
    std::vector<double> joint_reference_delta(num_joints_);
    kinematics_->convert_cartesian_deltas_to_joint_deltas(
        current_state_.positions, cartesian_delta_vec, params_.tool_frame_id,
        joint_reference_delta);

    new_joint_reference.positions.resize(num_joints_);
    for (std::size_t i = 0; i < num_joints_; i++) {
      new_joint_reference.positions[i] =
          current_state_.positions[i] + joint_reference_delta[i];
    }
  }

  write_state_to_hardware(new_joint_reference);

  last_tool_reference_ = new_tool_reference;

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

//==============================================================================
Eigen::Quaterniond Controller::expMapQuaternion(const Eigen::Vector3d& delta) {
  Eigen::Quaterniond q_delta;
  double theta_squared = delta.squaredNorm();
  if (theta_squared > Eigen::NumTraits<double>::dummy_precision()) {
    double theta = std::sqrt(theta_squared);
    q_delta.w() = std::cos(theta);
    q_delta.vec() = (std::sin(theta) / theta) * delta;
  } else {
    // taylor expansions around theta_squared==0
    q_delta.w() = 1.0 - 0.5 * theta_squared;
    q_delta.vec() = (1.0 - (1.0 / 6.0) * theta_squared) * delta;
  }

  return q_delta;
}

//==============================================================================
Eigen::Vector3d Controller::logMapQuaternion(const Eigen::Quaterniond& q) {
  if ((1.0 - q.squaredNorm()) < Eigen::NumTraits<double>::dummy_precision()) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "quaternion q must be approx. of unit length. 1.0 - "
                 "q.squaredNorm() is %f",
                 1.0 - q.squaredNorm());
    // todo(johntgz) throw error here?
  }

  // Implementation of the logarithmic map of SU(2) using atan.
  // We use atan2 instead of atan to enable the use of Eigen Autodiff with SU2:
  // atan2(y,x) is equivalent to atan(y/x) for x > 0. In our case x = w, the
  // real part of the quaternion.
  // With q = -q we chose the quaternion with positive real part.

  // todo(johntgz) better way to do this?
  const double sign_of_w = q.w() < 0.0 ? -1.0 : 1.0;
  const double abs_w = sign_of_w * q.w();
  const Eigen::Vector3d v = sign_of_w * q.vec();
  const double squared_norm_of_v = v.squaredNorm();

  Eigen::Vector3d delta;
  if (squared_norm_of_v > Eigen::NumTraits<double>::dummy_precision()) {
    const double norm_of_v = std::sqrt(squared_norm_of_v);
    delta = (std::atan2(norm_of_v, abs_w) / norm_of_v) * v;
  } else {
    // Perform taylor expansion at squared_norm_of_v == 0
    delta = (1.0 / abs_w - squared_norm_of_v / (3.0 * std::pow(abs_w, 3))) * v;
  }
  return delta;
}

//==============================================================================
Eigen::Quaterniond Controller::SphericalLinearInterpolation(
    const double& p, const Eigen::Quaterniond& q_a,
    const Eigen::Quaterniond& q_b) {
  double p0, p1;
  double cos_omega = q_a.dot(q_b);
  double abs_cos_omega = std::abs(cos_omega);

  // If angle is close to zero, perform only linear interpolation
  // else perform spherical linear interpolation
  if (abs_cos_omega > (1.0 - Eigen::NumTraits<double>::epsilon())) {
    p0 = 1.0 - p;
    p1 = p;
  } else {
    double omega = std::acos(abs_cos_omega);
    double sin_omega = sin(omega);
    p0 = sin((1 - p) * omega) / sin_omega;
    p1 = sin(p * omega) / sin_omega;
  }

  // Pick the right direction
  if (cos_omega < 0) {
    p1 = -p1;
  }

  return Eigen::Quaterniond(p0 * q_a.coeffs() + p1 * q_b.coeffs());
}

//==============================================================================
CartesianState Controller::IntegratePose(const CartesianState& pose,
                                         const double& control_frequency) {
  CartesianState new_pose = pose;

  // Integrate translation by one timestep
  new_pose.pose.translation() +=
      (1.0 / control_frequency) * pose.velocity.head<3>();

  // The orientation component requires the quaternion derivative:
  // qd = 0.5 * Q(q) * ad, where Q(q) is a matrix composed from the quaternion q
  // and ad is the angular velocity
  // Q(q) = [-q.x -q.y -q.z;
  //          q.w  q.z -q.y;
  //         -q.z  q.w  q.x;
  //          q.y -q.x  q.w]
  Eigen::Quaterniond q = pose.getPoseQuaternion();
  Eigen::Vector3d ad = pose.velocity.tail<3>();
  Eigen::Quaterniond qd;
  qd.w() = 0.5 * (-q.x() * ad(0) - q.y() * ad(1) - q.z() * ad(2));
  qd.x() = 0.5 * (q.w() * ad(0) + q.z() * ad(1) - q.y() * ad(2));
  qd.y() = 0.5 * (-q.z() * ad(0) + q.w() * ad(1) + q.x() * ad(2));
  qd.z() = 0.5 * (q.y() * ad(0) - q.x() * ad(1) + q.w() * ad(2));

  // Integrate quaternion by one timestep
  Eigen::Quaterniond q_new;
  q_new.w() = q.w() + (1.0 / control_frequency) * qd.w();
  q_new.vec() = q.vec() + (1.0 / control_frequency) * qd.vec();

  new_pose.setPoseQuaternion(q_new.normalized());

  return new_pose;
}

//==============================================================================
bool Controller::ClampReferenceToLimits(const CartesianLimits& limits,
                                        const uint8_t& mode,
                                        CartesianState& target_state,
                                        double soft_margin_meters,
                                        double soft_margin_radians) {
  bool mutated = false;

  bool clamp_pose =
      mode == TrajectoryGenerationMode::MODE_POSITION ||
      mode == TrajectoryGenerationMode::MODE_POSITION_AND_VELOCITY;

  bool scale_velocity =
      mode == TrajectoryGenerationMode::MODE_VELOCITY ||
      mode == TrajectoryGenerationMode::MODE_POSITION_AND_VELOCITY;

  Eigen::Vector3d new_translation = target_state.pose.translation();
  Eigen::Matrix<double, 6, 1> new_velocity = target_state.velocity;

  // Scale linear and angular velocity
  if (scale_velocity) {
    // Compute scaling factor for translational velocity
    double translational_scaling_factor = 1.0;
    for (int k = 0; k < 3; ++k) {
      double translational_scaling_factor_candidate = 1.0;
      if (new_velocity(k) > limits.max_translational_velocity(k)) {
        translational_scaling_factor_candidate =
            limits.max_translational_velocity(k) / new_velocity(k);
      } else if (new_velocity(k) < limits.min_translational_velocity(k)) {
        translational_scaling_factor_candidate =
            limits.min_translational_velocity(k) / new_velocity(k);
      }
      if (translational_scaling_factor_candidate <
          translational_scaling_factor) {
        translational_scaling_factor = translational_scaling_factor_candidate;
      }
    }

    // Apply scaling factor for translational velocity
    if (translational_scaling_factor < 0.0) {
      RCLCPP_ERROR(this->get_node()->get_logger(),
                   "Encountered negative scaling factor while adjusting the "
                   "translational velocity. Ensure that the limit interval is "
                   "valid. Defaulting twist to zero.");

      new_velocity.array() *= 0.0;
      mutated = true;

    } else if (translational_scaling_factor < 1.0) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                           1000, "Scaling translational velocity by %f",
                           translational_scaling_factor);

      new_velocity.head(3) *= translational_scaling_factor;
      mutated = true;
    }

    // Find scaling factor for rotational velocity and apply it.
    if (new_velocity.tail(3).norm() > limits.max_rotational_velocity) {
      double rotational_scaling_factor =
          limits.max_rotational_velocity / new_velocity.tail(3).norm();
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                           1000, "Scaling rotational velocity by %f",
                           rotational_scaling_factor);
      new_velocity.tail(3) *= rotational_scaling_factor;
      mutated = true;
    }
  }

  if (clamp_pose) {
    // Clamp translational components to limits

    // Get mask for elements that exceeded soft margins
    auto trans_exceed_max_margin_mask =
        (new_translation.array() >=
         (limits.max_translational_position.array() - soft_margin_meters));
    auto trans_exceed_min_margin_mask =
        (new_translation.array() <=
         (limits.min_translational_position.array() + soft_margin_meters));

    new_translation =
        new_translation.cwiseMin(limits.max_translational_position);
    new_translation =
        new_translation.cwiseMax(limits.min_translational_position);

    // If translational soft margins are violated, smoothly scale linear
    // velocity to zero based on the distance to the limits
    if (scale_velocity) {
      for (std::size_t i = 0; i < 3; ++i) {
        if (!trans_exceed_max_margin_mask(i) &&
            !trans_exceed_min_margin_mask(i)) {
          continue;
        }
        double scale_factor = 0.0;
        if (soft_margin_meters > 0.0) {
          // Compute the normalized distance relative to the start of the soft
          // margin
          double distance_from_soft_margin = 0.0;
          if (new_velocity(i) > 0.0) {
            distance_from_soft_margin =
                new_translation(i) -
                (limits.max_translational_position(i) - soft_margin_meters);

          } else if (new_velocity(i) < 0.0) {
            distance_from_soft_margin =
                (limits.min_translational_position(i) + soft_margin_meters) -
                new_translation(i);
          }

          double normalized_distance_from_soft_margin = std::clamp(
              distance_from_soft_margin / soft_margin_meters, 0.0, 1.0);
          // The bi-square function creates a smooth and differentiable
          // transition between 0 and 1.
          scale_factor = (1. - normalized_distance_from_soft_margin *
                                   normalized_distance_from_soft_margin) *
                         (1.0 - normalized_distance_from_soft_margin *
                                    normalized_distance_from_soft_margin);
        }

        new_velocity(i) *= scale_factor;
      }
    }

    // Clamp rotational components to limits

    // compute relative quaternion between current and reference quaternion
    Eigen::Quaterniond relative_quaternion =
        target_state.getPoseQuaternion() *
        limits.reference_quaternion_for_min_max.inverse();
    relative_quaternion.normalize();

    // The exponential log of the quaternion provides 0.5 * rotational_offset
    // from the reference quaternion. The 0.5 comes from the fact that
    // quaternions use rotation angles divided by 2 in sin() and cos()
    // functions. Thus the result of the logMapQuaternion needs to be multiplied
    // by 2.0.
    Eigen::Vector3d rotational_offset =
        2.0 * logMapQuaternion(relative_quaternion);
    Eigen::Vector3d new_rotational_offset = rotational_offset;

    // Get mask for elements that exceeded soft margins
    auto rot_exceed_max_margin_mask =
        new_rotational_offset.array() >=
        (limits.max_rotation_angle.array() - soft_margin_radians);
    auto rot_exceed_min_margin_mask =
        new_rotational_offset.array() <=
        (limits.min_rotation_angle.array() + soft_margin_radians);

    // Clamp rotational offsets
    new_rotational_offset =
        new_rotational_offset.cwiseMax(limits.min_rotation_angle);
    new_rotational_offset =
        new_rotational_offset.cwiseMin(limits.max_rotation_angle);

    // If rotational soft margins are violated, smoothly scale angular
    // velocity to zero based on the distance to the limits
    if (scale_velocity) {
      for (std::size_t i = 0; i < 3; i++) {
        if (!rot_exceed_max_margin_mask(i) && !rot_exceed_min_margin_mask(i)) {
          continue;
        }
        double scale_factor = 0.0;
        if (soft_margin_radians > 0.0) {
          // Compute the normalized distance relative to the start of the soft
          // margin
          double distance_from_soft_margin = 0.0;
          if (new_velocity(i + 3) > 0.0) {
            distance_from_soft_margin =
                new_rotational_offset(i) -
                (limits.max_rotation_angle(i) - soft_margin_radians);

          } else if (new_velocity(i + 3) < 0.0) {
            distance_from_soft_margin =
                (limits.min_rotation_angle(i) + soft_margin_radians) -
                new_rotational_offset(i);
          }

          double normalized_distance_from_soft_margin = std::clamp(
              distance_from_soft_margin / soft_margin_radians, 0.0, 1.0);
          // The bi-square function creates a smooth and differentiable
          // transition between 0 and 1.
          scale_factor = (1. - normalized_distance_from_soft_margin *
                                   normalized_distance_from_soft_margin) *
                         (1.0 - normalized_distance_from_soft_margin *
                                    normalized_distance_from_soft_margin);
        }
        new_velocity(i + 3) *= scale_factor;
      }
    }

    // todo(johtngz)
    // Update pose quaternion with the modified rotational offset.
    if (!rotational_offset.isApprox(new_rotational_offset)) {
      RCLCPP_WARN_STREAM_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Limit violation: Rotational offset clamped to "
              << new_rotational_offset);

      // Divide the rotational offset by 2.0 to apply expMapQuaternion
      // correctly.
      new_rotational_offset.array() /= 2.0;
      target_state.setPoseQuaternion(expMapQuaternion(new_rotational_offset) *
                                     limits.reference_quaternion_for_min_max);

      mutated = true;
    }

    if (!target_state.pose.translation().isApprox(new_translation)) {
      RCLCPP_WARN_STREAM_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Limit violation: Pose translation clamped to " << new_translation);

      target_state.pose.translation() = new_translation;
      mutated = true;
    }
    if (!target_state.velocity.isApprox(new_velocity)) {
      RCLCPP_WARN_STREAM_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Limit violation: Pose velocity clampted to " << new_velocity);

      target_state.velocity = new_velocity;
      mutated = true;
    }
  }

  return mutated;
}

bool Controller::UpdateReferenceLinearInterpolation(
    const CartesianState& last_reference, const CartesianState& target_state,
    const double remaining_time_to_target_seconds,
    const double control_frequency, const uint8_t& mode,
    CartesianState& new_reference) {
  bool interpolate_pose =
      mode == TrajectoryGenerationMode::MODE_POSITION ||
      mode == TrajectoryGenerationMode::MODE_POSITION_AND_VELOCITY;
  bool interpolate_velocity =
      mode == TrajectoryGenerationMode::MODE_VELOCITY ||
      mode == TrajectoryGenerationMode::MODE_POSITION_AND_VELOCITY;

  bool pose_only = mode == TrajectoryGenerationMode::MODE_POSITION;
  bool velocity_only = mode == TrajectoryGenerationMode::MODE_VELOCITY;

  if (!interpolate_pose && !interpolate_velocity) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unexpected trajectory generation mode. Please set to "
                 "either MODE_POSITION, MODE_VELOCITY or "
                 "MODE_POSITION_AND_VELOCITY");
    return false;
  }

  if (remaining_time_to_target_seconds > 0.0) {
    if (interpolate_pose) {
      // Linearly interpolate the translation
      new_reference.pose.translation() +=
          (target_state.pose.translation() -
           last_reference.pose.translation()) /
          (control_frequency * remaining_time_to_target_seconds);

      // for rotation, we utilise spherical linear interpolation (SLERP)
      auto p = 1.0 / (control_frequency * remaining_time_to_target_seconds);
      auto new_quaternion =
          SphericalLinearInterpolation(p, last_reference.getPoseQuaternion(),
                                       target_state.getPoseQuaternion());
      new_reference.setPoseQuaternion(new_quaternion);
    }
    if (interpolate_velocity) {
      // Linearly interpolate the velocity
      new_reference.velocity +=
          (target_state.velocity - last_reference.velocity) /
          (control_frequency * remaining_time_to_target_seconds);
    }
  } else {
    if (interpolate_pose) {
      // Hold the target position upon reaching the trajectory endpoint
      new_reference.pose = target_state.pose;
    }
    if (interpolate_velocity) {
      // Hold the target velocity upon reaching the trajectory endpoint
      new_reference.velocity = target_state.velocity;
    }
  }
  if (pose_only) {
    // Always set reference velocity to zero.
    new_reference.velocity.setZero();
  }
  if (velocity_only) {
    // Integrate reference pose by one timestep
    new_reference = IntegratePose(new_reference, control_frequency);
  }

  return true;
}

}  // namespace aic_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(aic_controller::Controller,
                       controller_interface::ControllerInterface)
