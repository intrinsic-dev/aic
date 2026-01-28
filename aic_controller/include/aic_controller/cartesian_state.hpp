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
#include "std_msgs/msg/header.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace aic_controller {

//==============================================================================
// Cartesian state with pose and velocity
struct CartesianState {
  Eigen::Isometry3d pose;
  Eigen::Matrix<double, 6, 1> velocity;
  std_msgs::msg::Header header;

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
   * @param header_msg
   */
  CartesianState(const geometry_msgs::msg::Pose& pose_msg,
                 const geometry_msgs::msg::Twist& velocity_msg,
                 const std_msgs::msg::Header& header_msg);

  /**
   * @brief Get quaternion of pose
   *
   * @return Eigen::Quaterniond
   */
  Eigen::Quaterniond get_pose_quaternion() const;

  /**
   * @brief Set quaternion of pose
   *
   * @param quaternion
   */
  void set_pose_quaternion(const Eigen::Quaterniond& quaternion);

  /**
   * @brief Get the pose vector in the form of (x, y, z, qx, qy, qz, qw) where
   * (x, y, z) is the translation and (qx, qy, qz, qw) are the quaternion
   * coefficients.
   *
   * @return Eigen::Matrix<double, 7, 1> Pose vector
   */
  Eigen::Matrix<double, 7, 1> get_pose_vector() const;
};

}  // namespace aic_controller

#endif  // AIC_CONTROLLER__CARTESIAN_STATE_HPP_
