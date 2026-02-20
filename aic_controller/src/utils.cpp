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

#include "aic_controller/utils.hpp"

//==============================================================================
namespace aic_controller {

//==============================================================================
namespace utils {

//==============================================================================
Eigen::Quaterniond exp_map_quaternion(const Eigen::Vector3d& delta) {
  Sophus::SO3d delta_so3 = Sophus::SO3d::exp(delta);

  return delta_so3.unit_quaternion();
}

//==============================================================================
Eigen::Vector3d log_map_quaternion(const Eigen::Quaterniond& q) {
  Sophus::SO3d so3 = Sophus::SO3d(q.normalized());

  return so3.log();
}

//==============================================================================
CartesianState integrate_pose(const CartesianState& pose,
                              const double& control_frequency) {
  CartesianState new_pose = pose;
  const double dt = 1.0 / control_frequency;

  // 1. Represent current pose as a SE3 object (Transformation Matrix)
  Sophus::SE3d T_current(pose.pose.matrix());

  // 2. Represent velocity as a 6D tangent vector (Twist)
  // Sophus SE3 order: [v_x, v_y, v_z, omega_x, omega_y, omega_z]
  Eigen::Matrix<double, 6, 1> velocity = pose.velocity;

  // 3. Integrate the pose:
  // NewPose = OldPose * exp(velocity * dt)
  Sophus::SE3d T_new = T_current * Sophus::SE3d::exp(velocity * dt);

  // 4. Update the cartesian state
  new_pose.pose.matrix() = T_new.matrix();

  return new_pose;
}

//==============================================================================
void wrench_msg_to_eigen(const geometry_msgs::msg::Wrench& msg,
                         Eigen::Matrix<double, 6, 1>& wrench_eigen) {
  wrench_eigen(0) = msg.force.x;
  wrench_eigen(1) = msg.force.y;
  wrench_eigen(2) = msg.force.z;
  wrench_eigen(3) = msg.torque.x;
  wrench_eigen(4) = msg.torque.y;
  wrench_eigen(5) = msg.torque.z;
}

//==============================================================================
void eigen_to_wrench_msg(const Eigen::Matrix<double, 6, 1>& wrench_eigen,
                         geometry_msgs::msg::Wrench& msg) {
  msg.force.x = wrench_eigen(0);
  msg.force.y = wrench_eigen(1);
  msg.force.z = wrench_eigen(2);
  msg.torque.x = wrench_eigen(3);
  msg.torque.y = wrench_eigen(4);
  msg.torque.z = wrench_eigen(5);
}

}  // namespace utils

}  // namespace aic_controller
