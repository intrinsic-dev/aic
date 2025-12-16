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
Eigen::Quaterniond expMapQuaternion(const Eigen::Vector3d& delta) {
  Eigen::Quaterniond q_delta;
  double theta_squared = delta.squaredNorm();
  if (theta_squared > Eigen::NumTraits<double>::dummy_precision()) {
    double theta = std::sqrt(theta_squared);
    q_delta.w() = std::cos(theta);
    q_delta.vec() = (std::sin(theta) / theta) * delta;
  } else {
    // taylor expansions around theta_squared==0
    q_delta.w() = 1.0 - 0.5 * theta_squared;
    q_delta.vec() = (1.0 - (1.0 / 6.0) * theta_squared) * delta;
  }

  return q_delta;
}

//==============================================================================
Eigen::Vector3d logMapQuaternion(const Eigen::Quaterniond& q_in) {
  Eigen::Quaterniond q = q_in;
  if ((1.0 - q.squaredNorm()) >= Eigen::NumTraits<double>::dummy_precision()) {
    q.normalize();
  }

  // Implementation of the logarithmic map of SU(2) using atan.
  // We use atan2 instead of atan to enable the use of Eigen Autodiff with SU2:
  // atan2(y,x) is equivalent to atan(y/x) for x > 0. In our case x = w, the
  // real part of the quaternion.
  // With q = -q we chose the quaternion with positive real part.

  // todo(johntgz) better way to write these operations?
  const double sign_of_w = q.w() < 0.0 ? -1.0 : 1.0;
  const double abs_w = sign_of_w * q.w();
  const Eigen::Vector3d v = sign_of_w * q.vec();
  const double squared_norm_of_v = v.squaredNorm();

  Eigen::Vector3d delta;
  if (squared_norm_of_v > Eigen::NumTraits<double>::dummy_precision()) {
    const double norm_of_v = std::sqrt(squared_norm_of_v);
    delta = (std::atan2(norm_of_v, abs_w) / norm_of_v) * v;
  } else {
    // Perform taylor expansion at squared_norm_of_v == 0
    delta = (1.0 / abs_w - squared_norm_of_v / (3.0 * std::pow(abs_w, 3))) * v;
  }
  return delta;
}

//==============================================================================
CartesianState IntegratePose(const CartesianState& pose,
                             const double& control_frequency) {
  CartesianState new_pose = pose;

  // Integrate translation by one timestep
  new_pose.pose.translation() +=
      (1.0 / control_frequency) * pose.velocity.head<3>();

  // The orientation component requires the quaternion derivative:
  // qd = 0.5 * Q(q) * ad, where Q(q) is a matrix composed from the quaternion q
  // and ad is the angular velocity
  // Q(q) = [-q.x -q.y -q.z;
  //          q.w  q.z -q.y;
  //         -q.z  q.w  q.x;
  //          q.y -q.x  q.w]
  Eigen::Quaterniond q = pose.getPoseQuaternion();
  Eigen::Vector3d ad = pose.velocity.tail<3>();
  Eigen::Quaterniond qd;
  qd.w() = 0.5 * (-q.x() * ad(0) - q.y() * ad(1) - q.z() * ad(2));
  qd.x() = 0.5 * (q.w() * ad(0) + q.z() * ad(1) - q.y() * ad(2));
  qd.y() = 0.5 * (-q.z() * ad(0) + q.w() * ad(1) + q.x() * ad(2));
  qd.z() = 0.5 * (q.y() * ad(0) - q.x() * ad(1) + q.w() * ad(2));

  // Integrate quaternion by one timestep
  Eigen::Quaterniond q_new;
  q_new.w() = q.w() + (1.0 / control_frequency) * qd.w();
  q_new.vec() = q.vec() + (1.0 / control_frequency) * qd.vec();

  new_pose.setPoseQuaternion(q_new.normalized());

  return new_pose;
}

}  // namespace utils

}  // namespace aic_controller
