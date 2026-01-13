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

#ifndef AIC_CONTROLLER__ACTIONS__JOINT_IMPEDANCE_ACTION_HPP_
#define AIC_CONTROLLER__ACTIONS__JOINT_IMPEDANCE_ACTION_HPP_

#include <Eigen/Core>

#include "joint_limits/joint_limits.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Interfaces
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

//==============================================================================
namespace aic_controller {
using trajectory_msgs::msg::JointTrajectoryPoint;

//==============================================================================
struct JointImpedanceParameters {
  // This is required for fixed-size Eigen types to precent segmentation faults
  // resulting from memory alignment issues
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Parameters for impedance control.
  Eigen::VectorXd stiffness_vector;
  Eigen::VectorXd damping_vector;
  Eigen::VectorXd feedforward_torques;
  // Values used in interpolating the feedforward torque
  Eigen::VectorXd interpolator_min_value;
  Eigen::VectorXd interpolator_max_value;
  Eigen::VectorXd interpolator_max_step_size;

  JointImpedanceParameters() {}

  explicit JointImpedanceParameters(int num_joints)
      : stiffness_vector(Eigen::VectorXd::Zero(num_joints)),
        damping_vector(Eigen::VectorXd::Zero(num_joints)),
        feedforward_torques(Eigen::VectorXd::Zero(num_joints)),
        interpolator_min_value(Eigen::VectorXd::Zero(num_joints)),
        interpolator_max_value(Eigen::VectorXd::Zero(num_joints)),
        interpolator_max_step_size(Eigen::VectorXd::Zero(num_joints)) {}
};

//==============================================================================
class JointImpedanceAction {
 public:
  JointImpedanceAction(std::size_t num_joints);

  /**
   * @brief Configure the joint impedance action with joint limits and
   * relevant node interfaces
   *
   * @param joint_limits Joint limits on joint position, velocity, accelration
   * @param logging_if Node interface for logging
   * @return true Configuration successful
   * @return false Configuration failed
   */
  [[nodiscard]]
  bool configure(
      const std::vector<joint_limits::JointLimits>& joint_limits,
      const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr&
          logging_if,
      const rclcpp::node_interfaces::NodeClockInterface::SharedPtr& clock_if);

  /**
   * @brief Generates a target joint torque given the joint state errors and
   * control parameters.
   *
   * @param joint_position_error Position error between target and current joint
   * states
   * @param joint_velocity_error Velocity error between target and current joint
   * states
   * @param params Impedance control parameters
   * @param new_joint_reference Joint target torque
   * @return true Computation successful
   * @return false Computation failed
   */
  bool compute(const Eigen::VectorXd& joint_position_error,
               const Eigen::VectorXd& joint_velocity_error,
               const JointImpedanceParameters& params,
               JointTrajectoryPoint& new_joint_reference);

 private:
  // Number of robot joints
  const std::size_t num_joints_;
  std::vector<joint_limits::JointLimits> joint_limits_;

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_if_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_if_;
};

}  // namespace aic_controller

#endif  // AIC_CONTROLLER__ACTIONS__JOINT_IMPEDANCE_ACTION_HPP_
