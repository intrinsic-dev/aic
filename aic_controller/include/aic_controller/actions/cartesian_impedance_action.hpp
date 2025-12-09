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

#ifndef AIC_CONTROLLER__ACTIONS__CARTESIAN_IMPEDANCE_ACTION_HPP_
#define AIC_CONTROLLER__ACTIONS__CARTESIAN_IMPEDANCE_ACTION_HPP_

#include <Eigen/Core>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Interfaces
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "aic_controller/cartesian_state.hpp"

//==============================================================================
namespace aic_controller {
using trajectory_msgs::msg::JointTrajectoryPoint;

//==============================================================================
struct CartesianImpedanceParameters {};

//==============================================================================
class CartesianImpedanceAction {
 public:
  CartesianImpedanceAction(std::size_t num_joints);

  /**
   * @brief Loads the inverse kinematic plugin given the robot description
   *
   * @param node lifecycle node
   * @param robot_description robot description used by kinematics plugin
   * @return controller_interface::return_type
   */
	[[nodiscard]]
  bool Configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                 const std::string& robot_description);

  /**
   * @brief Generates a target joint torque with the control torque given the
   * nullspace, tool goal, sensed joint position and velocity, and control
   * parameters.
   *
   * @param tool_target Goal state of tool control frame
   * @param current_state Current joint states
   * @param impedance_params Impedance controller parameters
   * @param joint_limits Joint limits on joint position, velocity, accelration
   * and further derivatives
   * @return JointTrajectoryPoint Joint target torque
   */
  JointTrajectoryPoint Compute(const CartesianState tool_target,
                          const JointTrajectoryPoint& current_state
                          // const CartesianImpedanceParameters& impedance_params
                          // const JointLimits& joint_limits
  );

 private:
  // Number of robot joints
  const std::size_t num_joints_;
};

}  // namespace aic_controller

#endif  // AIC_CONTROLLER__ACTIONS__CARTESIAN_IMPEDANCE_ACTION_HPP_
