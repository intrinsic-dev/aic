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

#ifndef AIC_CONTROLLER__JOINT_STATE_HPP_
#define AIC_CONTROLLER__JOINT_STATE_HPP_

#include <Eigen/Core>

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace aic_controller {
using trajectory_msgs::msg::JointTrajectoryPoint;

//==============================================================================
// Joint state of the robot with position and velocity vectors
struct JointState {
  std::size_t num_joints_;
  Eigen::VectorXd positions;
  Eigen::VectorXd velocities;

  /**
   * @brief Default constructor.
   *
   */
  JointState();

  /**
   * @brief Constructs JointState with size of position and velocity vectors set
   * to num_joints_ and their values as 0.
   *
   * @param num_joints_ Number of joints
   */
  JointState(const std::size_t num_joints);

  /**
   * @brief Constructs JointState from a JointTrajectoryPoint message.
   * If the size of the 'positions' and 'velocities' field in the message does
   * not match num_joints_, their values will default to zero.
   *
   * @param msg JointTrajectoryPoint ROS Message
   * @param msg num_joints Number of joints
   */
  JointState(JointTrajectoryPoint& msg, const std::size_t num_joints);
};

}  // namespace aic_controller

#endif  // AIC_CONTROLLER__JOINT_STATE_HPP_
