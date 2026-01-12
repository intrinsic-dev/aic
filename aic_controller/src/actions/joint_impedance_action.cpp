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

#include "aic_controller/actions/joint_impedance_action.hpp"

namespace aic_controller {

//==============================================================================
JointImpedanceAction::JointImpedanceAction(std::size_t num_joints)
    : num_joints_(num_joints) {}

//==============================================================================
bool JointImpedanceAction::configure(
    const std::vector<joint_limits::JointLimits>& joint_limits,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_if,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr& clock_if) {
  joint_limits_ = joint_limits;
  logging_if_ = logging_if;
  clock_if_ = clock_if;

  return true;
}

//==============================================================================
bool JointImpedanceAction::compute(const Eigen::VectorXd& joint_position_error,
                                   const Eigen::VectorXd& joint_velocity_error,
                                   const JointImpedanceParameters& params,
                                   JointTrajectoryPoint& new_joint_reference) {
  // Compute the control torques u using the equation
  // u = K * (x_des - x) + D * (v_des - v) + u_f
  // where D is damping, K is stiffness and u_f the feedforward torques

  Eigen::VectorXd target_torque =
      params.stiffness_vector.cwiseProduct(joint_position_error) +
      params.damping_vector.cwiseProduct(joint_velocity_error) +
      params.feedforward_torques;

  // Clamp to joint torque limits
  for (std::size_t k = 0; k < num_joints_; ++k) {
    target_torque(k) =
        std::clamp(target_torque(k), -joint_limits_[k].max_effort,
                   joint_limits_[k].max_effort);
  }

  new_joint_reference.effort.resize(num_joints_);
  Eigen::VectorXd::Map(new_joint_reference.effort.data(),
                       new_joint_reference.effort.size()) = target_torque;

  return true;
}

}  // namespace aic_controller
