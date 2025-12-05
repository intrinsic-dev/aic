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

#include "aic_controller/actions/cartesian_impedance_action.hpp"

namespace aic_controller {

std::unique_ptr<CartesianImpedanceAction>
CartesianImpedanceAction::Create(
    const std::shared_ptr<aic_controller::ParamListener>& param_listener) {
  auto params = param_listener->get_params();

  std::size_t ndof = params.joints.size();
  if (ndof < 1) {
    RCLCPP_ERROR(
        rclcpp::get_logger("CartesianImpedanceAction"),
        "Unable to create CartesianImpedanceAction with num_joints < 1");
    return nullptr;
  }

  return std::make_unique<aic_controller::CartesianImpedanceAction>(ndof,
                                                             std::move(params));
}

CartesianImpedanceAction::CartesianImpedanceAction(
    unsigned int ndof, aic_controller::Params params)
    : ndof_(ndof), params_(params) {}

controller_interface::return_type CartesianImpedanceAction::Configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
    const std::string& robot_description) {
  // UNIMPLEMENTED
  // Load the differential IK plugin given the robot_description

  return controller_interface::return_type::OK;
}

Eigen::VectorXd CartesianImpedanceAction::Compute(
    const geometry_msgs::msg::Pose tool_pose,
    const geometry_msgs::msg::Twist tool_vel,
    const CartesianImpedanceParameters& impedance_params,
    const JointLimits& joint_limits) {
  // UNIMPLEMENTED
  // Compute control wrench using the control law

  return Eigen::VectorXd();
}

bool CartesianImpedanceAction::Update(
    const JointTrajectoryPoint& current_sensed) {
  // UNIMPLEMENTED
  // Compute the end-effector cartesian pose estimate using forward kinematics.
  // Compute the jacobian

  return false;
}

}  // namespace aic_controller
