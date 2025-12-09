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

#ifndef AIC_CONTROLLER__CARTESIAN_STATE_HPP_
#define AIC_CONTROLLER__CARTESIAN_STATE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/accel.hpp"

#include "tf2_eigen/tf2_eigen.hpp"

namespace aic_controller {

//==============================================================================
// Cartesian state with pose and velocity
struct CartesianState {

  Eigen::Isometry3d pose;
  Eigen::Matrix<double, 6, 1> velocity;

  /**
   * @brief Default constructor with pose set to identity, velocity and
   * acceleration values set to zero.
   *
   */
  CartesianState();

  /**
   * @brief Construct a CartesianState from ROS messages
   *
   * @param pose_msg
   * @param velocity_msg
   * @param acceleration_msg
   */
  CartesianState(
		const geometry_msgs::msg::Pose& pose_msg,
		const geometry_msgs::msg::Twist& velocity_msg);
};

}  // namespace aic_controller

#endif  // AIC_CONTROLLER__CARTESIAN_STATE_HPP_
