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

//==============================================================================
CartesianImpedanceAction::CartesianImpedanceAction(std::size_t num_joints)
    : num_joints_(num_joints) {}

//==============================================================================
bool CartesianImpedanceAction::Configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
    const std::string& robot_description) {
  // UNIMPLEMENTED
  // Load the differential IK plugin given the robot_description
  (void)node;
  (void)robot_description;

  return true;
}

//==============================================================================
JointTrajectoryPoint CartesianImpedanceAction::Compute(
    const CartesianState tool_target,
    const JointTrajectoryPoint& current_state
    // const CartesianImpedanceParameters& impedance_params
    // const JointLimits& joint_limits
) {
  // UNIMPLEMENTED
  // Compute control wrench using the control law
	(void)tool_target;
	(void)current_state;

  JointTrajectoryPoint joint_target;
  joint_target.effort.assign(num_joints_, 0.0);

  return joint_target;
}

}  // namespace aic_controller
