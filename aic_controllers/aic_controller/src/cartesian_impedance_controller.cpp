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

#include "aic_controller/cartesian_impedance_controller.hpp"

namespace aic {

std::unique_ptr<CartesianImpedanceController>
CartesianImpedanceController::Create(
    const std::shared_ptr<aic_controller::ParamListener>& param_listener) {
  auto params = param_listener->get_params();

  std::size_t ndof = params.joints.size();
  if (ndof < 1) {
    RCLCPP_ERROR(
        rclcpp::get_logger("CartesianImpedanceController"),
        "Unable to create CartesianImpedanceController with num_joints < 1");
    return nullptr;
  }

  return std::make_unique<aic::CartesianImpedanceController>(ndof,
                                                             std::move(params));
}

CartesianImpedanceController::CartesianImpedanceController(
    unsigned int ndof, aic_controller::Params params)
    : ndof_(ndof), params_(params) {}

controller_interface::return_type CartesianImpedanceController::Configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node) {
  // UNIMPLEMENTED
  // Load the differential IK plugin given the robot_description

  return controller_interface::return_type::OK;
}

Eigen::Matrix<double, 6, 1> CartesianImpedanceController::Compute(
    const CartState& tool_target,
    const CartesianImpedanceParameters& impedance_params,
    const JointLimits& joint_limits) {
  // UNIMPLEMENTED
  // Compute control wrench using the control law

  return Eigen::Matrix<double, 6, 1>::Zero();
}

bool CartesianImpedanceController::Update(const JointState& current_sensed) {
  // UNIMPLEMENTED
  // Compute the end-effector cartesian pose estimate using forward kinematics.
  // Compute the jacobian

  return false;
}

}  // namespace aic
