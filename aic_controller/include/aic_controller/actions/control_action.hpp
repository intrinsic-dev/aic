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

#ifndef AIC_CONTROLLER__ACTIONS__CONTROL_ACTION_HPP_
#define AIC_CONTROLLER__ACTIONS__CONTROL_ACTION_HPP_

#include "aic_controller/actions/control_parameters.hpp"
#include "joint_limits/joint_limits.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Interfaces
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

//==============================================================================
namespace aic_controller {
using trajectory_msgs::msg::JointTrajectoryPoint;

//==============================================================================
class ControlAction {
 public:
  ControlAction(std::size_t num_joints);

  virtual ~ControlAction() = default;

  [[nodiscard]]
  virtual bool configure(
      const std::vector<joint_limits::JointLimits>& joint_limits,
      const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr&
          logging_if,
      const rclcpp::node_interfaces::NodeClockInterface::SharedPtr& clock_if);

  virtual bool compute(const ControlParameters& params,
                       JointTrajectoryPoint& new_joint_reference) = 0;

 private:
  // Number of robot joints
  const std::size_t num_joints_;
  std::vector<joint_limits::JointLimits> joint_limits_;

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_if_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_if_;
};

}  // namespace aic_controller

#endif  // AIC_CONTROLLER__ACTIONS__CONTROL_ACTION_HPP_
