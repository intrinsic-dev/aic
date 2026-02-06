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
namespace aic_controller {

//==============================================================================
Controller::Controller()
    : param_listener_(nullptr),
      num_joints_(0),
      control_mode_(ControlMode::Invalid),
      target_mode_(TargetMode::Invalid),
      cartesian_impedance_action_(nullptr),
      feedforward_wrench_at_tip_(Eigen::Matrix<double, 6, 1>::Zero()),
      sensed_wrench_at_tip_(Eigen::Matrix<double, 6, 1>::Zero()),
      joint_impedance_action_(nullptr),
      gravity_compensation_action_(nullptr),
      motion_update_sub_(nullptr),
      joint_motion_update_sub_(nullptr),
      state_publisher_rt_(nullptr),
      motion_update_received_(false),
      last_commanded_state_(std::nullopt),
      target_state_(std::nullopt),
      joint_target_state_(std::nullopt),
      last_tool_pose_error_(Eigen::Matrix<double, 6, 1>::Zero()),
      time_to_target_seconds_(0.0),
      remaining_time_to_target_seconds_(0.0),
      kinematics_loader_(nullptr),
      kinematics_(nullptr),
      force_torque_sensor_(nullptr) {
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
  }
  for (const auto& joint : params_.joints) {
    state_interfaces_config_names.push_back(joint + "/" +
                                            hardware_interface::HW_IF_VELOCITY);
  }

  if (control_mode_ == ControlMode::Impedance) {
    auto ft_state_interfaces =
        force_torque_sensor_->get_state_interface_names();
    state_interfaces_config_names.insert(state_interfaces_config_names.end(),
                                         ft_state_interfaces.begin(),
                                         ft_state_interfaces.end());
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

  joint_impedance_action_ = std::make_unique<JointImpedanceAction>(num_joints_);

  gravity_compensation_action_ =
      std::make_unique<GravityCompensationAction>(num_joints_);

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

  if (params_.target_mode == "cartesian") {
    RCLCPP_INFO(
        get_node()->get_logger(),
        "Target mode set to Cartesian. Accepting MotionUpdate targets.");
    target_mode_ = TargetMode::Cartesian;
  } else if (params_.target_mode == "joint") {
    RCLCPP_INFO(
        get_node()->get_logger(),
        "Target mode set to joint. Accepting JointMotionUpdate targets.");
    target_mode_ = TargetMode::Joint;
  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unsupported target mode. Please set control_mode to either "
                 "'cartesian' or 'joint'");
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
      "~/pose_commands", reliable_qos,
      [this](const MotionUpdate::SharedPtr msg) {
        if (get_node()->get_current_state().id() !=
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
                               *get_node()->get_clock(), 1000,
                               "Controller is not in ACTIVE lifecycle state, "
                               "ignoring MotionUpdate message.");

          return;
        }

        if (target_mode_ != TargetMode::Cartesian) {
          RCLCPP_WARN(get_node()->get_logger(),
                      "Please switch to Cartesian target mode before sending "
                      "MotionUpdate targets. Ignoring "
                      "MotionUpdate message.");

          return;
        }

        if (msg->trajectory_generation_mode.mode ==
            TrajectoryGenerationMode::MODE_UNSPECIFIED) {
          RCLCPP_WARN(
              get_node()->get_logger(),
              "The trajectory_generation_mode is set to MODE_UNSPECIFIED. "
              "Please set to either MODE_POSITION or MODE_VELOCITY. Ignoring "
              "MotionUpdate message.");

          return;
        }

        // Currently, only targets with frame_id "base_link" and "gripper/tcp"
        // are supported
        if (msg->header.frame_id != "base_link" &&
            msg->header.frame_id != "gripper/tcp") {
          RCLCPP_WARN(get_node()->get_logger(),
                      "Only accepting targets with frame_id 'base_link' or "
                      "'gripper/tcp'. "
                      "Ignoring MotionUpdate message");

          return;
        }

        motion_update_rt_.set(*msg);
        motion_update_received_ = true;
      });

  joint_motion_update_sub_ =
      this->get_node()->create_subscription<JointMotionUpdate>(
          "~/joint_commands", reliable_qos,
          [this](const JointMotionUpdate::SharedPtr msg) {
            if (get_node()->get_current_state().id() !=
                lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
              RCLCPP_WARN_THROTTLE(
                  get_node()->get_logger(), *get_node()->get_clock(), 1000,
                  "Controller is not in ACTIVE lifecycle state. "
                  "Ignoring JointMotionUpdate message.");

              return;
            }

            if (target_mode_ != TargetMode::Joint) {
              RCLCPP_WARN(get_node()->get_logger(),
                          "Please switch to Joint target mode before sending "
                          "JointMotionUpdate targets. Ignoring "
                          "JointMotionUpdate message.");

              return;
            }

            if (msg->target_stiffness.size() != num_joints_) {
              RCLCPP_WARN(get_node()->get_logger(),
                          "The size of the target_stiffness does not match the "
                          "number of joints. Expected size %ld. "
                          "Ignoring JointMotionUpdate message.",
                          num_joints_);

              return;
            }
            if (msg->target_damping.size() != num_joints_) {
              RCLCPP_WARN(
                  get_node()->get_logger(),
                  "The size of the target_damping does not match the number "
                  "of joints. Expected size %ld. "
                  "Ignoring JointMotionUpdate message.",
                  num_joints_);

              return;
            }
            if (!msg->target_feedforward_torque.empty()) {
              if (msg->target_feedforward_torque.size() != num_joints_) {
                RCLCPP_WARN(get_node()->get_logger(),
                            "The size of the target_feedforward_torque does "
                            "not match the number "
                            "of joints. Expected size %ld. "
                            "Ignoring JointMotionUpdate message.",
                            num_joints_);

                return;
              }
            }
            if (msg->trajectory_generation_mode.mode ==
                TrajectoryGenerationMode::MODE_UNSPECIFIED) {
              RCLCPP_WARN(get_node()->get_logger(),
                          "The trajectory_generation_mode is set to "
                          "MODE_UNSPECIFIED. "
                          "Please set to either MODE_POSITION or "
                          "MODE_VELOCITY. Ignoring JointMotionUpdate "
                          "message.");

              return;
            }
            if (msg->trajectory_generation_mode.mode ==
                TrajectoryGenerationMode::MODE_POSITION) {
              if (msg->target_state.positions.size() != num_joints_) {
                RCLCPP_WARN(get_node()->get_logger(),
                            "The size of the target_state does not match the "
                            "number of joints. Expected size %ld. "
                            "Ignoring JointMotionUpdate message.",
                            num_joints_);

                return;
              }
            }
            if (msg->trajectory_generation_mode.mode ==
                TrajectoryGenerationMode::MODE_VELOCITY) {
              if (msg->target_state.velocities.size() != num_joints_) {
                RCLCPP_WARN(get_node()->get_logger(),
                            "The size of the target_state does not match the "
                            "number of joints. Expected size %ld. "
                            "Ignoring JointMotionUpdate message.",
                            num_joints_);

                return;
              }
            }

            joint_motion_update_rt_.set(*msg);
            motion_update_received_ = true;
          });

  change_target_mode_srv_ = this->get_node()->create_service<ChangeTargetMode>(
      "~/change_target_mode",
      [this](const std::shared_ptr<ChangeTargetMode::Request> request,
             std::shared_ptr<ChangeTargetMode::Response> response) {
        // todo(johntgz) Add check and reject request if there is an on-going
        // execution.

        if (request->target_mode ==
            ChangeTargetMode::Request::TARGET_MODE_CARTESIAN) {
          if (target_mode_ == TargetMode::Cartesian) {
            RCLCPP_INFO(get_node()->get_logger(),
                        "Controller is already in Cartesian target mode.");
            response->success = true;

            return;
          }

          RCLCPP_INFO(get_node()->get_logger(),
                      "Received request to switch target mode to "
                      "CARTESIAN_TARGET_MODE.");

          // Reset any previously set JointMotionUpdate target
          joint_motion_update_ = aic_controller::JointMotionUpdate();
          joint_motion_update_rt_.try_set(joint_motion_update_);
          motion_update_received_ = false;
          joint_target_state_ = std::nullopt;
          last_tool_reference_ = current_tool_state_;

          target_mode_ = TargetMode::Cartesian;

          response->success = true;
        } else if (request->target_mode ==
                   ChangeTargetMode::Request::TARGET_MODE_JOINT) {
          if (target_mode_ == TargetMode::Joint) {
            RCLCPP_INFO(get_node()->get_logger(),
                        "Controller is already in Joint target mode.");
            response->success = true;

            return;
          }

          RCLCPP_INFO(
              get_node()->get_logger(),
              "Received request to switch target mode to JOINT_TARGET_MODE.");

          // Reset any previously set MotionUpdate target
          motion_update_ = aic_controller::MotionUpdate();
          motion_update_rt_.try_set(motion_update_);
          motion_update_received_ = false;
          target_state_ = std::nullopt;
          last_joint_reference_ = JointState(current_state_, num_joints_);

          target_mode_ = TargetMode::Joint;

          response->success = true;
        } else {
          RCLCPP_WARN(get_node()->get_logger(),
                      "Invalid target mode requested. Please choose either "
                      "CARTESIAN_TARGET_MODE or JOINT_TARGET_MODE");

          response->success = false;
        }
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

  urdf::Model urdf_model;
  if (!urdf_model.initString(this->get_robot_description())) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to parse URDF from robot_description string");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize joint limits
  joint_limits_.resize(num_joints_);
  for (std::size_t i = 0; i < num_joints_; ++i) {
    auto urdf_joint = urdf_model.getJoint(params_.joints[i]);
    if (!urdf_joint) {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint %s not found in the URDF",
                   params_.joints[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (!joint_limits::getJointLimits(urdf_joint, joint_limits_[i])) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Unable to get joint limit for joint %s",
                   params_.joints[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  if (control_mode_ == ControlMode::Impedance) {
    // Initialize force torque sensor
    force_torque_sensor_ =
        std::make_unique<semantic_components::ForceTorqueSensor>(
            params_.force_torque_sensor.name);

    // Initialize GravityCompensationAction if enabled
    if (params_.impedance.gravity_compensation) {
      if (!gravity_compensation_action_->configure(
              urdf_model, params_.kinematics.base, params_.kinematics.tip,
              get_node()->get_node_logging_interface())) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Failed to configure GravityCompensationAction!");

        return controller_interface::CallbackReturn::ERROR;
      }
    }

    // Validate impedance control parameters
    for (const double& gain : params_.impedance.pose_error_integrator.gain) {
      if (gain < 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Invalid impedance pose_error_integrator gain. "
                     "Required: >= 0. Received: %f",
                     gain);
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    for (const double& bound : params_.impedance.pose_error_integrator.bound) {
      if (bound < 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Invalid impedance pose_error_integrator bound. "
                     "Required: >= 0. Received: %f",
                     bound);
        return controller_interface::CallbackReturn::ERROR;
      }
    }

    // Populate Cartesian impedance control parameters
    CartesianImpedanceParameters resized_impedance_params(num_joints_);
    impedance_params_ = resized_impedance_params;

    // Set default parameters for stiffness and damping matrices
    impedance_params_.stiffness_matrix =
        Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
            params_.impedance.default_values.control_stiffness.data(),
            params_.impedance.default_values.control_stiffness.size())
            .asDiagonal();
    impedance_params_.damping_matrix =
        Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
            params_.impedance.default_values.control_damping.data(),
            params_.impedance.default_values.control_damping.size())
            .asDiagonal();

    // set desired nullspace configuration, stiffness and damping
    impedance_params_.nullspace_goal = Eigen::Map<const Eigen::VectorXd>(
        params_.impedance.nullspace.target_configuration.data(),
        static_cast<Eigen::Index>(num_joints_));
    impedance_params_.nullspace_stiffness = Eigen::Map<const Eigen::VectorXd>(
        params_.impedance.nullspace.stiffness.data(),
        static_cast<Eigen::Index>(num_joints_));
    impedance_params_.nullspace_damping = Eigen::Map<const Eigen::VectorXd>(
        params_.impedance.nullspace.damping.data(),
        static_cast<Eigen::Index>(num_joints_));

    impedance_params_.activation_percentage =
        params_.impedance.activation_percentage;
    impedance_params_.maximum_wrench =
        Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
            params_.impedance.maximum_wrench.data());

    impedance_params_.feedforward_interpolation_wrench_min =
        Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
            params_.impedance.feedforward_interpolation.min_wrench.data());
    impedance_params_.feedforward_interpolation_wrench_max =
        Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
            params_.impedance.feedforward_interpolation.max_wrench.data());
    impedance_params_.feedforward_interpolation_max_wrench_dot =
        Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
            params_.impedance.feedforward_interpolation.max_wrench_dot.data());

    impedance_params_.pose_error_integrator_gain =
        Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
            params_.impedance.pose_error_integrator.gain.data());
    impedance_params_.pose_error_integrator_bound =
        Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
            params_.impedance.pose_error_integrator.bound.data());

    impedance_params_.offset_wrench =
        Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
            params_.impedance.default_values.offset_wrench.data());

    if (!populate_cartesian_limits(params_, cartesian_limits_)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Error populating cartesian limits from parameters.");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (!cartesian_impedance_action_->configure(
            joint_limits_, get_node()->get_node_logging_interface(),
            get_node()->get_node_clock_interface())) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to configure CartesianImpedanceAction.");

      return controller_interface::CallbackReturn::ERROR;
    }

    // Validate joint impedance control parameters
    if (params_.joint_impedance.default_values.control_stiffness.size() !=
        num_joints_) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "The size of the "
                   "joint_impedance.control_stiffness.control_damping does "
                   "not match the number of joints. Expected size %ld.",
                   num_joints_);

      return controller_interface::CallbackReturn::ERROR;
    }
    if (params_.joint_impedance.default_values.control_damping.size() !=
        num_joints_) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "The size of the joint_impedance.default_values.control_damping does "
          "not match the number of joints. Expected size %ld.",
          num_joints_);

      return controller_interface::CallbackReturn::ERROR;
    }

    if (params_.joint_impedance.interpolator.min_value.size() != num_joints_) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "The size of the joint_impedance.interpolator.min_value does "
          "not match the number of joints. Expected size %ld.",
          num_joints_);

      return controller_interface::CallbackReturn::ERROR;
    }
    if (params_.joint_impedance.interpolator.max_value.size() != num_joints_) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "The size of the joint_impedance.interpolator.max_value does "
          "not match the number of joints. Expected size %ld.",
          num_joints_);

      return controller_interface::CallbackReturn::ERROR;
    }
    if (params_.joint_impedance.interpolator.max_step_size.size() !=
        num_joints_) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "The size of the joint_impedance.interpolator.max_step_size does "
          "not match the number of joints. Expected size %ld.",
          num_joints_);

      return controller_interface::CallbackReturn::ERROR;
    }

    // Populate joint impedance control parameters
    JointImpedanceParameters resized_joint_impedance_params(num_joints_);
    joint_impedance_params_ = resized_joint_impedance_params;

    // Set default parameters for stiffness and damping matrices
    joint_impedance_params_.stiffness_vector =
        Eigen::Map<const Eigen::VectorXd>(
            params_.joint_impedance.default_values.control_stiffness.data(),
            static_cast<Eigen::Index>(params_.joint_impedance.default_values
                                          .control_stiffness.size()));

    joint_impedance_params_.damping_vector = Eigen::Map<const Eigen::VectorXd>(
        params_.joint_impedance.default_values.control_damping.data(),
        static_cast<Eigen::Index>(
            params_.joint_impedance.default_values.control_damping.size()));

    joint_impedance_params_.interpolator_min_value =
        Eigen::Map<const Eigen::VectorXd>(
            params_.joint_impedance.interpolator.min_value.data(),
            static_cast<Eigen::Index>(
                params_.joint_impedance.interpolator.min_value.size()));

    joint_impedance_params_.interpolator_max_value =
        Eigen::Map<const Eigen::VectorXd>(
            params_.joint_impedance.interpolator.max_value.data(),
            static_cast<Eigen::Index>(
                params_.joint_impedance.interpolator.max_value.size()));

    joint_impedance_params_.interpolator_max_step_size =
        Eigen::Map<const Eigen::VectorXd>(
            params_.joint_impedance.interpolator.max_step_size.data(),
            static_cast<Eigen::Index>(
                params_.joint_impedance.interpolator.max_step_size.size()));

    if (!joint_impedance_action_->configure(
            joint_limits_, get_node()->get_node_logging_interface(),
            get_node()->get_node_clock_interface())) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to configure JointImpedanceAction.");

      return controller_interface::CallbackReturn::ERROR;
    }
  }

  state_publisher_ = get_node()->create_publisher<ControllerState>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
  state_publisher_rt_ =
      std::make_unique<realtime_tools::RealtimePublisher<ControllerState>>(
          state_publisher_);

  return controller_interface::CallbackReturn::SUCCESS;
}

//==============================================================================
controller_interface::CallbackReturn Controller::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (control_mode_ == ControlMode::Impedance) {
    force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  }

  // read and initialize current joint states
  current_state_.positions.assign(num_joints_, 0.0);
  current_state_.velocities.assign(num_joints_, 0.0);
  read_state_from_hardware(current_state_, current_tool_state_,
                           sensed_wrench_at_tip_);
  for (const auto& val : current_state_.positions) {
    if (std::isnan(val)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to read joint positions from the hardware.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  last_tool_reference_ = current_tool_state_;
  last_joint_reference_ = JointState(current_state_, num_joints_);

  motion_update_ = aic_controller::MotionUpdate();
  motion_update_rt_.try_set(motion_update_);

  joint_motion_update_ = aic_controller::JointMotionUpdate();
  joint_motion_update_rt_.try_set(joint_motion_update_);

  motion_update_received_ = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

//==============================================================================
controller_interface::CallbackReturn Controller::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (control_mode_ == ControlMode::Impedance) {
    force_torque_sensor_->release_interfaces();
  }
  release_interfaces();

  motion_update_ = aic_controller::MotionUpdate();
  motion_update_rt_.try_set(motion_update_);

  joint_motion_update_ = aic_controller::JointMotionUpdate();
  joint_motion_update_rt_.try_set(joint_motion_update_);

  motion_update_received_ = false;

  target_state_ = std::nullopt;
  joint_target_state_ = std::nullopt;

  // Reset feedforward wrenches
  feedforward_wrench_at_tip_.setZero();
  impedance_params_.feedforward_wrench.setZero();
  joint_impedance_params_.feedforward_torques.setZero();

  return controller_interface::CallbackReturn::SUCCESS;
}

//==============================================================================
controller_interface::CallbackReturn Controller::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  param_listener_.reset();
  motion_update_sub_.reset();
  joint_motion_update_sub_.reset();
  change_target_mode_srv_.reset();
  state_publisher_rt_.reset();
  state_publisher_.reset();

  cartesian_impedance_action_.reset();
  joint_impedance_action_.reset();
  gravity_compensation_action_.reset();
  force_torque_sensor_.reset();

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
  read_state_from_hardware(current_state_, current_tool_state_,
                           sensed_wrench_at_tip_);

  auto current_joint_state = JointState(current_state_, num_joints_);

  // read user commands
  if (motion_update_received_) {
    if (target_mode_ == TargetMode::Cartesian) {
      auto command_op = motion_update_rt_.try_get();
      if (command_op.has_value()) {
        motion_update_ = command_op.value();

        auto latest_target_state =
            CartesianState(motion_update_.pose, motion_update_.velocity,
                           motion_update_.header);

        if (motion_update_.trajectory_generation_mode.mode ==
            TrajectoryGenerationMode::MODE_POSITION) {
          if (motion_update_.header.frame_id == "gripper/tcp") {
            // Only update the target pose if there is a change in the message
            // timestamp.
            if (!target_state_.has_value() ||
                target_state_.value().header.stamp !=
                    latest_target_state.header.stamp) {
              // transform target pose from "gripper/tcp" frame to "base_link"
              // frame
              latest_target_state.pose.linear() =
                  current_tool_state_.pose.linear().inverse() *
                  latest_target_state.pose.linear();
              latest_target_state.pose.translation() =
                  current_tool_state_.pose.linear().inverse() *
                      latest_target_state.pose.translation() +
                  current_tool_state_.pose.translation();

              target_state_ = latest_target_state;
            }
          } else {
            target_state_ = latest_target_state;
          }
        } else if (motion_update_.trajectory_generation_mode.mode ==
                   TrajectoryGenerationMode::MODE_VELOCITY) {
          // In velocity mode, we set the target pose as the current pose
          // so that the velocity targets are applied relative to the
          // TCP frame
          latest_target_state.pose = current_tool_state_.pose;
          if (motion_update_.header.frame_id == "base_link") {
            // Transform target velocity from base frame into the TCP frame
            latest_target_state.velocity.head<3>() =
                current_tool_state_.pose.rotation() *
                latest_target_state.velocity.head<3>();
            latest_target_state.velocity.tail<3>() =
                current_tool_state_.pose.rotation() *
                latest_target_state.velocity.tail<3>();
          }
          target_state_ = latest_target_state;
        }
      }
    } else if (target_mode_ == TargetMode::Joint) {
      auto command_op = joint_motion_update_rt_.try_get();
      if (command_op.has_value()) {
        joint_motion_update_ = command_op.value();

        joint_target_state_ =
            JointState(joint_motion_update_.target_state, num_joints_);
      }
    }
  }

  // If target_state_ has a value, then we set the new tool reference to that of
  // the target and interpolate it.
  // Else, maintain the current position of the robot.

  CartesianState new_tool_reference = last_tool_reference_;
  JointState new_joint_reference = last_joint_reference_;

  if (target_mode_ == TargetMode::Cartesian && target_state_.has_value()) {
    // Clamp the target states to stay within limits
    if (clamp_reference_to_limits(
            cartesian_limits_, motion_update_.trajectory_generation_mode.mode,
            target_state_.value())) {
      RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Limit violation: Cartesian target has been clamped to limits");
    }

    // Apply linear interpolation to the target_state_ to obtain a new
    // reference. Linear interpolation should support MODE_POSITION and
    // MODE_VELOCITY
    if (!update_reference_linear_interpolation(
            last_tool_reference_, target_state_.value(),
            remaining_time_to_target_seconds_, params_.control_frequency,
            motion_update_.trajectory_generation_mode.mode,
            new_tool_reference)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Linear interpolation of Cartesian target failed");
      return controller_interface::return_type::ERROR;
    }

  } else if (target_mode_ == TargetMode::Joint &&
             joint_target_state_.has_value()) {
    // Clamp the target states to stay within limits
    if (clamp_joint_reference_to_limits(
            joint_limits_, joint_motion_update_.trajectory_generation_mode.mode,
            joint_target_state_.value())) {
      RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Limit violation: Joint target has been clamped to limits");
    }

    // Apply linear interpolation to the target_state_ to obtain a new
    // reference. Linear interpolation should support MODE_POSITION and
    // MODE_VELOCITY
    if (!update_joint_reference_linear_interpolation(
            last_joint_reference_, joint_target_state_.value(),
            remaining_time_to_target_seconds_, params_.control_frequency,
            joint_motion_update_.trajectory_generation_mode.mode,
            new_joint_reference)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Linear interpolation of joint target failed");
      return controller_interface::return_type::ERROR;
    }
  }

  // Decrement the remaining time to target
  if (remaining_time_to_target_seconds_ > 0) {
    remaining_time_to_target_seconds_ -= 1. / params_.control_frequency;
    if (remaining_time_to_target_seconds_ < 0)
      remaining_time_to_target_seconds_ = 0;
  }

  // Compute controls
  JointTrajectoryPoint new_joint_command;

  if (control_mode_ == ControlMode::Impedance) {
    new_joint_command.effort.assign(num_joints_, 0.0);

    if (motion_update_received_) {
      interpolate_impedance_parameters();
    }

    if (target_mode_ == TargetMode::Cartesian) {
      // Compute the tool pose and velocity error between the current and
      // target tool state.
      Eigen::Matrix<double, 7, 1> current_pose_vec =
          current_tool_state_.get_pose_vector();
      Eigen::Matrix<double, 7, 1> target_pose_vec =
          new_tool_reference.get_pose_vector();
      Eigen::Matrix<double, 6, 1> tool_pose_error;
      if (!kinematics_->calculate_frame_difference(
              current_pose_vec, target_pose_vec, 1.0, tool_pose_error)) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Failed to calculate frame difference between current and "
                     "target tool frame");
        return controller_interface::return_type::ERROR;
      }
      last_tool_pose_error_ = tool_pose_error;

      // Transform target velocity from TCP frame into base frame
      Eigen::Matrix<double, 6, 1> new_tool_reference_base_frame;
      new_tool_reference_base_frame.head<3>() =
          current_tool_state_.pose.rotation().inverse() *
          new_tool_reference.velocity.head<3>();
      new_tool_reference_base_frame.tail<3>() =
          current_tool_state_.pose.rotation().inverse() *
          new_tool_reference.velocity.tail<3>();

      Eigen::Matrix<double, 6, 1> tool_vel_error =
          new_tool_reference_base_frame - current_tool_state_.velocity;

      // Calculate the current Jacobian.
      Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, num_joints_);
      if (!kinematics_->calculate_jacobian(current_joint_state.positions,
                                           params_.kinematics.tip, jacobian)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to calculate jacobian");
        return controller_interface::return_type::ERROR;
      }

      // todo(johntgz) replace current_State_ with current_joint_state
      if (!cartesian_impedance_action_->compute(
              tool_pose_error, tool_vel_error, current_state_, jacobian,
              impedance_params_, new_joint_command)) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Cartesian Impedance Action failed to compute controls!");
        return controller_interface::return_type::ERROR;
      }

    } else if (target_mode_ == TargetMode::Joint) {
      // Compute joint position and velocity error between current and target
      // state
      Eigen::VectorXd joint_position_error =
          new_joint_reference.positions - current_joint_state.positions;
      Eigen::VectorXd joint_velocity_error =
          new_joint_reference.velocities - current_joint_state.velocities;

      if (!joint_impedance_action_->compute(
              joint_position_error, joint_velocity_error,
              joint_impedance_params_, new_joint_command)) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Cartesian Impedance Action failed to compute controls!");
        return controller_interface::return_type::ERROR;
      }
    }

    if (params_.impedance.gravity_compensation) {
      Eigen::VectorXd gravity_compensation_torques;
      if (!gravity_compensation_action_->compute(
              current_joint_state.positions, gravity_compensation_torques)) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Failed to calculate torques for gravity compensation!");

        return controller_interface::return_type::ERROR;
      }
      for (std::size_t i = 0; i < num_joints_; ++i) {
        new_joint_command.effort[i] += gravity_compensation_torques[i];
      }
    }

  } else if (control_mode_ == ControlMode::Admittance) {
    // UNIMPLEMENTED

    RCLCPP_ERROR(get_node()->get_logger(),
                 "Admittance control is unimplemented");

    return controller_interface::return_type::ERROR;
  }

  write_state_to_hardware(new_joint_command);

  last_tool_reference_ = new_tool_reference;
  last_joint_reference_ = new_joint_reference;

  populate_controller_state(state_msg_);
  state_publisher_rt_->try_publish(state_msg_);

  return controller_interface::return_type::OK;
}

//==============================================================================
void Controller::read_state_from_hardware(
    JointTrajectoryPoint& state_current, CartesianState& tool_state_current,
    Eigen::Matrix<double, 6, 1>& sensed_wrench_at_tip) {
  // Set state_current to last commanded state if any of the hardware interface
  // values are NaN
  bool nan_position = false;
  bool nan_velocity = false;

  for (std::size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind) {
    const auto state_current_position_op =
        state_interfaces_[joint_ind].get_optional();
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

  // Use forward kinematics to update the current cartesian state of tool frame
  if (!kinematics_->calculate_link_transform(state_current.positions,
                                             params_.kinematics.tip,
                                             tool_state_current.pose)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unable to compute current cartesian state of tool frame");
  }

  // Retrieve the cartesian velocity of the tool frame
  std::vector<double> cartesian_velocity(6);
  if (!kinematics_->convert_joint_deltas_to_cartesian_deltas(
          state_current.positions, state_current.velocities,
          params_.kinematics.tip, cartesian_velocity)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unable to compute current cartesian velocity of tool frame");
    std::fill(cartesian_velocity.begin(), cartesian_velocity.end(), 0.0);
  }
  tool_state_current.velocity =
      Eigen::Map<const Eigen::Matrix<double, 6, 1>>(cartesian_velocity.data());

  if (control_mode_ == ControlMode::Impedance) {
    // Retrieve readings from force torque sensor and in the event of NaN
    // readings, set all values to zero.
    auto ft_forces = force_torque_sensor_->get_forces();
    auto ft_torques = force_torque_sensor_->get_torques();

    bool nan_ft_value = false;
    nan_ft_value |= std::any_of(ft_forces.begin(), ft_forces.end(),
                                [](double val) { return std::isnan(val); });
    nan_ft_value |= std::any_of(ft_torques.begin(), ft_torques.end(),
                                [](double val) { return std::isnan(val); });
    if (nan_ft_value) {
      RCLCPP_ERROR(this->get_node()->get_logger(),
                   "Read NaN value from force-torque sensor. Setting sensed "
                   "values to zero.");
      sensed_wrench_at_tip.setZero();
    } else {
      sensed_wrench_at_tip.head<3>() =
          Eigen::Map<const Eigen::Vector3d>(ft_forces.data());
      sensed_wrench_at_tip.tail<3>() =
          Eigen::Map<const Eigen::Vector3d>(ft_torques.data());
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
bool Controller::populate_cartesian_limits(const aic_controller::Params& params,
                                           CartesianLimits& limits) {
  for (std::size_t i = 0; i < 3; ++i) {
    if (params.clamp_to_limits.min_translational_position[i] >=
        params.clamp_to_limits.max_translational_position[i]) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Error setting cartesian limits at index [%ld]. Minimum "
                   "translational position >= maximum translational "
                   "position: %f >= %f",
                   i, params.clamp_to_limits.min_translational_position[i],
                   params.clamp_to_limits.max_translational_position[i]);
      return false;
    }
    if (params.clamp_to_limits.min_rotation_angle[i] >=
        params.clamp_to_limits.max_rotation_angle[i]) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Error setting cartesian limits at index [%ld]. Minimum "
                   "rotation angle >= maximum rotation angle: %f >= %f",
                   i, params.clamp_to_limits.min_rotation_angle[i],
                   params.clamp_to_limits.max_rotation_angle[i]);
      return false;
    }
  }

  limits.min_translational_position = Eigen::Map<const Eigen::VectorXd>(
      params.clamp_to_limits.min_translational_position.data(),
      static_cast<Eigen::Index>(
          params.clamp_to_limits.min_translational_position.size()));

  limits.max_translational_position = Eigen::Map<const Eigen::VectorXd>(
      params.clamp_to_limits.max_translational_position.data(),
      static_cast<Eigen::Index>(
          params.clamp_to_limits.max_translational_position.size()));

  limits.min_translational_velocity = Eigen::Map<const Eigen::VectorXd>(
      params.clamp_to_limits.min_translational_velocity.data(),
      static_cast<Eigen::Index>(
          params.clamp_to_limits.min_translational_velocity.size()));

  limits.max_translational_velocity = Eigen::Map<const Eigen::VectorXd>(
      params.clamp_to_limits.max_translational_velocity.data(),
      static_cast<Eigen::Index>(
          params.clamp_to_limits.max_translational_velocity.size()));

  limits.min_translational_velocity = Eigen::Map<const Eigen::VectorXd>(
      params.clamp_to_limits.min_translational_velocity.data(),
      static_cast<Eigen::Index>(
          params.clamp_to_limits.min_translational_velocity.size()));

  limits.min_rotation_angle = Eigen::Map<const Eigen::VectorXd>(
      params.clamp_to_limits.min_rotation_angle.data(),
      static_cast<Eigen::Index>(
          params.clamp_to_limits.min_rotation_angle.size()));

  limits.max_rotation_angle = Eigen::Map<const Eigen::VectorXd>(
      params.clamp_to_limits.max_rotation_angle.data(),
      static_cast<Eigen::Index>(
          params.clamp_to_limits.max_rotation_angle.size()));

  limits.max_rotational_velocity =
      params.clamp_to_limits.max_rotational_velocity;

  return true;
}

//==============================================================================
void Controller::populate_controller_state(ControllerState& controller_state) {
  controller_state.header.stamp = get_node()->now();

  controller_state.tcp_pose = tf2::toMsg(current_tool_state_.pose);
  controller_state.tcp_velocity = tf2::toMsg(current_tool_state_.velocity);

  controller_state.reference_tcp_pose = tf2::toMsg(last_tool_reference_.pose);

  std::copy(last_tool_pose_error_.data(),
            last_tool_pose_error_.data() + last_tool_pose_error_.size(),
            controller_state.tcp_error.begin());

  if (last_commanded_state_.has_value()) {
    controller_state.reference_joint_state = last_commanded_state_.value();
  }
}

//==============================================================================
bool Controller::clamp_reference_to_limits(const CartesianLimits& limits,
                                           const uint8_t& mode,
                                           CartesianState& target_state,
                                           double soft_margin_meters,
                                           double soft_margin_radians) {
  bool mutated = false;

  bool clamp_pose = mode == TrajectoryGenerationMode::MODE_POSITION;

  bool scale_velocity = mode == TrajectoryGenerationMode::MODE_VELOCITY;

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

    } else if (translational_scaling_factor < 1.0) {
      RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Limit Violation: Scaling translational velocity by %f",
          translational_scaling_factor);

      new_velocity.head(3) *= translational_scaling_factor;
    }

    // Find scaling factor for rotational velocity and apply it.
    if (new_velocity.tail(3).norm() > limits.max_rotational_velocity) {
      double rotational_scaling_factor =
          limits.max_rotational_velocity / new_velocity.tail(3).norm();
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                           1000, "Scaling rotational velocity by %f",
                           rotational_scaling_factor);
      new_velocity.tail(3) *= rotational_scaling_factor;
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
        new_translation.cwiseMin(limits.max_translational_position)
            .cwiseMax(limits.min_translational_position);

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
        target_state.get_pose_quaternion() *
        limits.reference_quaternion_for_min_max.inverse();

    // Compute the logarithmic map of the unit quaternion to obtain the
    // corresponding tangent vector.
    Eigen::Vector3d rotational_offset =
        utils::log_map_quaternion(relative_quaternion);

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
        new_rotational_offset.cwiseMax(limits.min_rotation_angle)
            .cwiseMin(limits.max_rotation_angle);

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

    if (!rotational_offset.isApprox(new_rotational_offset)) {
      RCLCPP_WARN_STREAM_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Limit violation: Rotational offset clamped to "
              << new_rotational_offset.transpose());
      // Compute the exponential map of the rotational offset tangent vector to
      // obtain the unit quaternion
      Eigen::Quaterniond new_quaternion =
          utils::exp_map_quaternion(new_rotational_offset);
      target_state.set_pose_quaternion(new_quaternion *
                                       limits.reference_quaternion_for_min_max);

      mutated = true;
    }

    if (!target_state.pose.translation().isApprox(new_translation)) {
      RCLCPP_WARN_STREAM_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Limit violation: Pose translation clamped to "
              << new_translation.transpose());

      target_state.pose.translation() = new_translation;
      mutated = true;
    }
    if (!target_state.velocity.isApprox(new_velocity)) {
      RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(),
                                  *get_node()->get_clock(), 1000,
                                  "Limit violation: Pose velocity clampted to "
                                      << new_velocity.transpose());

      target_state.velocity = new_velocity;
      mutated = true;
    }
  }

  return mutated;
}

//==============================================================================
bool Controller::clamp_joint_reference_to_limits(
    const std::vector<JointLimits>& limits, const uint8_t& mode,
    JointState& target_state, double soft_margin_radians) {
  bool mutated = false;

  bool clamp_pose = mode == TrajectoryGenerationMode::MODE_POSITION;

  bool scale_velocity = mode == TrajectoryGenerationMode::MODE_VELOCITY;

  Eigen::VectorXd new_positions = target_state.positions;
  Eigen::VectorXd new_velocities = target_state.velocities;

  // Scale linear and angular velocity
  if (scale_velocity) {
    // Compute scaling factor for joint velocity
    double scaling_factor = 1.0;

    for (std::size_t k = 0; k < std::size_t(new_velocities.size()); ++k) {
      double scaling_factor_candidate = 1.0;
      if (new_velocities(k) > limits[k].max_velocity) {
        scaling_factor_candidate = limits[k].max_velocity / new_velocities(k);
      } else if (new_velocities(k) < -limits[k].max_velocity) {
        scaling_factor_candidate = -limits[k].max_velocity / new_velocities(k);
      }
      if (scaling_factor_candidate < scaling_factor) {
        scaling_factor = scaling_factor_candidate;
      }
    }

    // Apply scaling factor for joint velocity
    if (scaling_factor < 0.0) {
      RCLCPP_ERROR(this->get_node()->get_logger(),
                   "Encountered negative scaling factor while scaling the "
                   "joint velocities. Scaling velocities to zero.");

      new_velocities *= 0.0;
    } else if (scaling_factor < 1.0) {
      RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Limit Violation: Scaling translational velocity by %f",
          scaling_factor);

      new_velocities *= scaling_factor;
    }
  }

  if (clamp_pose) {
    // Clamp joint positions to limits.

    // If position soft margins are violated, smoothly scale linear
    // velocity to zero based on the distance to the limits.
    for (std::size_t i = 0; i < std::size_t(new_positions.size()); ++i) {
      if (new_positions(i) >= (limits[i].max_position - soft_margin_radians) ||
          new_positions(i) <= (limits[i].min_position + soft_margin_radians)) {
        new_positions(i) = std::clamp(new_positions(i), limits[i].min_position,
                                      limits[i].max_position);

        if (scale_velocity) {
          double scale_factor = 0.0;
          if (soft_margin_radians > 0.0) {
            // Compute the normalized distance relative to the start of the soft
            // margin
            double distance_from_soft_margin = 0.0;
            if (new_velocities(i) > 0.0) {
              distance_from_soft_margin =
                  new_positions(i) -
                  (limits[i].max_position - soft_margin_radians);

            } else if (new_velocities(i) < 0.0) {
              distance_from_soft_margin =
                  (limits[i].min_position + soft_margin_radians) -
                  new_positions(i);
            }

            double normalized_distance_from_soft_margin = std::clamp(
                distance_from_soft_margin / soft_margin_radians, 0.0, 1.0);
            // The bi-square function creates a smooth and differentiable
            // transition between 0 and 1.
            scale_factor = (1.0 - normalized_distance_from_soft_margin *
                                      normalized_distance_from_soft_margin) *
                           (1.0 - normalized_distance_from_soft_margin *
                                      normalized_distance_from_soft_margin);
          }

          new_velocities(i) *= scale_factor;
        }
      }
    }

    if (!target_state.positions.isApprox(new_positions)) {
      RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(),
                                  *get_node()->get_clock(), 1000,
                                  "Limit violation: Joint positions clamped to "
                                      << new_positions.transpose());

      target_state.positions = new_positions;
      mutated = true;
    }
    if (!target_state.velocities.isApprox(new_velocities)) {
      RCLCPP_WARN_STREAM_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Limit violation: Joint velocities clampted to "
              << new_velocities.transpose());

      target_state.velocities = new_velocities;
      mutated = true;
    }
  }

  return mutated;
}

//==============================================================================
bool Controller::update_reference_linear_interpolation(
    const CartesianState& last_reference, const CartesianState& target_state,
    const double remaining_time_to_target_seconds,
    const double control_frequency, const uint8_t& mode,
    CartesianState& new_reference) {
  bool interpolate_pose = mode == TrajectoryGenerationMode::MODE_POSITION;
  bool interpolate_velocity = mode == TrajectoryGenerationMode::MODE_VELOCITY;

  if (!interpolate_pose && !interpolate_velocity) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unexpected trajectory generation mode. Please set to "
                 "either MODE_POSITION or MODE_VELOCITY");
    return false;
  }

  if (remaining_time_to_target_seconds > 0.0) {
    if (interpolate_pose) {
      // Linearly interpolate the translation
      new_reference.pose.translation() +=
          (target_state.pose.translation() -
           last_reference.pose.translation()) /
          (control_frequency * remaining_time_to_target_seconds);

      // For interpolating rotation, we utilise spherical linear interpolation
      // (SLERP) so as to keep the angular velocity constant
      double t = 1.0 / (control_frequency * remaining_time_to_target_seconds);
      Eigen::Quaterniond new_quaternion =
          last_reference.get_pose_quaternion()
              .slerp(t, target_state.get_pose_quaternion())
              .normalized();

      new_reference.set_pose_quaternion(new_quaternion);
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
  if (interpolate_pose) {
    // Always set reference velocity to zero.
    new_reference.velocity.setZero();
  }
  if (interpolate_velocity) {
    // Integrate reference pose by one timestep
    new_reference = utils::integrate_pose(new_reference, control_frequency);
    // Clamp new_reference to limits after integration
    clamp_reference_to_limits(cartesian_limits_,
                              TrajectoryGenerationMode::MODE_POSITION,
                              new_reference);
  }

  return true;
}

//==============================================================================
bool Controller::update_joint_reference_linear_interpolation(
    const JointState& last_reference, const JointState& target_state,
    const double remaining_time_to_target_seconds,
    const double control_frequency, const uint8_t& mode,
    JointState& new_reference) {
  bool interpolate_position = mode == TrajectoryGenerationMode::MODE_POSITION;
  bool interpolate_velocity = mode == TrajectoryGenerationMode::MODE_VELOCITY;

  if (!interpolate_position && !interpolate_velocity) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unexpected trajectory generation mode. Please set to "
                 "either MODE_POSITION or MODE_VELOCITY");
    return false;
  }

  if (remaining_time_to_target_seconds > 0.0) {
    if (interpolate_position) {
      // Linearly interpolate the joint positions
      new_reference.positions +=
          (target_state.positions - last_reference.positions) /
          (control_frequency * remaining_time_to_target_seconds);
    }
    if (interpolate_velocity) {
      // Linearly interpolate the velocity
      new_reference.velocities +=
          (target_state.velocities - last_reference.velocities) /
          (control_frequency * remaining_time_to_target_seconds);
      new_reference.positions += target_state.velocities / control_frequency;
    }
  } else {
    if (interpolate_position) {
      // Hold the target position upon reaching the trajectory endpoint
      new_reference.positions = target_state.positions;
    }
    if (interpolate_velocity) {
      // Hold the target velocity upon reaching the trajectory endpoint
      new_reference.velocities = target_state.velocities;
    }
  }
  if (interpolate_position) {
    // Always set reference velocity to zero.
    new_reference.velocities.setZero();
  }
  if (interpolate_velocity) {
    // Integrate reference pose by one timestep
    new_reference.positions += target_state.velocities / control_frequency;
    // Clamp new_reference to limits after integration
    clamp_joint_reference_to_limits(
        joint_limits_, TrajectoryGenerationMode::MODE_POSITION, new_reference);
  }

  return true;
}

//==============================================================================
void Controller::interpolate_impedance_parameters() {
  if (target_mode_ == TargetMode::Cartesian) {
    // We use exponential smoothing to interpolate the stiffness and damping
    // matrices with the equation:
    //
    //   S_(n+1) = (1-c) * S_n + c * S_target
    //
    // where n+1 is the next control iteration
    impedance_params_.stiffness_matrix *=
        1. - params_.impedance.stiffness_smoothing_constant;
    impedance_params_.stiffness_matrix +=
        params_.impedance.stiffness_smoothing_constant *
        Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
            motion_update_.target_stiffness.data());

    impedance_params_.damping_matrix *=
        1. - params_.impedance.damping_smoothing_constant;
    impedance_params_.damping_matrix +=
        params_.impedance.damping_smoothing_constant *
        Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
            motion_update_.target_damping.data());

    // Clamp feedforward target wrench to limits
    Eigen::Matrix<double, 6, 1> target_wrench;
    utils::wrench_msg_to_eigen(motion_update_.feedforward_wrench_at_tip,
                               target_wrench);
    const Eigen::Matrix<double, 6, 1> clamped_target_wrench =
        target_wrench
            .cwiseMin(impedance_params_.feedforward_interpolation_wrench_max)
            .cwiseMax(impedance_params_.feedforward_interpolation_wrench_min);

    // Interpolate from current wrench to clamped_target_wrench
    Eigen::Matrix<double, 6, 1> next_wrench = feedforward_wrench_at_tip_;
    for (int i = 0; i < 6; ++i) {
      if (clamped_target_wrench(i) > feedforward_wrench_at_tip_(i)) {
        next_wrench(i) =
            feedforward_wrench_at_tip_(i) +
            (impedance_params_.feedforward_interpolation_max_wrench_dot(i) *
             (1.0 / params_.control_frequency));
        next_wrench(i) = std::min(clamped_target_wrench(i), next_wrench(i));
      } else if (clamped_target_wrench(i) < feedforward_wrench_at_tip_(i)) {
        next_wrench(i) =
            feedforward_wrench_at_tip_(i) -
            (impedance_params_.feedforward_interpolation_max_wrench_dot(i) *
             (1.0 / params_.control_frequency));
        next_wrench(i) = std::max(clamped_target_wrench(i), next_wrench(i));
      } else {
        next_wrench(i) = feedforward_wrench_at_tip_(i);
      }
    }
    feedforward_wrench_at_tip_ = next_wrench;

    // Compute the total wrench at the tool tip
    // Force control via feedforward_wrench and wrench_feedback_gains.
    Eigen::Matrix<double, 6, 1> wrench_feedback_gains_at_tip;
    utils::wrench_msg_to_eigen(motion_update_.wrench_feedback_gains_at_tip,
                               wrench_feedback_gains_at_tip);
    Eigen::Matrix<double, 6, 1> total_wrench_at_tip =
        feedforward_wrench_at_tip_ +
        wrench_feedback_gains_at_tip.cwiseProduct(feedforward_wrench_at_tip_ -
                                                  sensed_wrench_at_tip_);

    // todo(johntgz) should the rotation be inverted?
    //  Rotate wrench at tool tip into base frame.
    impedance_params_.feedforward_wrench.head<3>() =
        current_tool_state_.pose.rotation() * total_wrench_at_tip.head<3>();
    impedance_params_.feedforward_wrench.tail<3>() =
        current_tool_state_.pose.rotation() * total_wrench_at_tip.tail<3>();

  } else if (target_mode_ == TargetMode::Joint) {
    // We use exponential smoothing to interpolate the stiffness and damping
    // vectors with the equation:
    //   S_(n+1) = (1-c) * S_n + c * S_target
    //
    // where n+1 is the next control iteration
    joint_impedance_params_.stiffness_vector *=
        (1. - params_.impedance.stiffness_smoothing_constant);
    joint_impedance_params_.stiffness_vector +=
        (params_.impedance.stiffness_smoothing_constant *
         Eigen::Map<const Eigen::VectorXd>(
             joint_motion_update_.target_stiffness.data(),
             static_cast<Eigen::Index>(num_joints_)));

    joint_impedance_params_.damping_vector *=
        (1. - params_.impedance.damping_smoothing_constant);
    joint_impedance_params_.damping_vector +=
        (params_.impedance.damping_smoothing_constant *
         Eigen::Map<const Eigen::VectorXd>(
             joint_motion_update_.target_damping.data(),
             static_cast<Eigen::Index>(num_joints_)));

    // Update the feedforward torque if a feedforward target is provided. Else,
    // default to zero feedforward torque.
    Eigen::VectorXd target_feedforward_torque =
        Eigen::VectorXd::Zero(num_joints_);
    if (!joint_motion_update_.target_feedforward_torque.empty()) {
      target_feedforward_torque = Eigen::Map<Eigen::VectorXd>(
          joint_motion_update_.target_feedforward_torque.data(),
          static_cast<Eigen::Index>(
              joint_motion_update_.target_feedforward_torque.size()));
    }

    // Interpolate from current feedforward torques to target feedforward
    // torque.
    target_feedforward_torque =
        target_feedforward_torque
            .cwiseMin(joint_impedance_params_.interpolator_max_value)
            .cwiseMax(joint_impedance_params_.interpolator_min_value);

    Eigen::VectorXd delta_torque =
        target_feedforward_torque - joint_impedance_params_.feedforward_torques;
    delta_torque =
        delta_torque
            .cwiseMin(joint_impedance_params_.interpolator_max_step_size)
            .cwiseMax(-joint_impedance_params_.interpolator_max_step_size);

    joint_impedance_params_.feedforward_torques += delta_torque;
  }

  return;
}

}  // namespace aic_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(aic_controller::Controller,
                       controller_interface::ControllerInterface)
