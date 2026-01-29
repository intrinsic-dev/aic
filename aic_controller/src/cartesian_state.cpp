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

#include "aic_controller/cartesian_state.hpp"

namespace aic_controller {

//==============================================================================
CartesianState::CartesianState()
    : pose(Eigen::Isometry3d::Identity()),
      velocity(Eigen::Matrix<double, 6, 1>::Zero()) {};

//==============================================================================
CartesianState::CartesianState(const geometry_msgs::msg::Pose& pose_msg,
                               const geometry_msgs::msg::Twist& velocity_msg,
                               const std_msgs::msg::Header& header_msg) {
  tf2::fromMsg(pose_msg, pose);
  tf2::fromMsg(velocity_msg, velocity);
  header = header_msg;
}

//==============================================================================
Eigen::Quaterniond CartesianState::get_pose_quaternion() const {
  return Eigen::Quaterniond(pose.linear());
}

//==============================================================================
void CartesianState::set_pose_quaternion(const Eigen::Quaterniond& quaternion) {
  pose.linear() = quaternion.toRotationMatrix();
}

//==============================================================================
Eigen::Matrix<double, 7, 1> CartesianState::get_pose_vector() const {
  Eigen::Matrix<double, 7, 1> pose_vec;
  pose_vec.head<3>() = pose.translation();
  pose_vec.tail<4>() = get_pose_quaternion().coeffs();

  return pose_vec;
}

}  // namespace aic_controller
