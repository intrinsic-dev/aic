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

#include "aic_controller/cartesian_limits.hpp"

namespace aic_controller {

//==============================================================================
CartesianLimits::CartesianLimits()
    : min_translational_position(Eigen::Vector3d::Zero()),
      max_translational_position(Eigen::Vector3d::Zero()),
      min_translational_velocity(Eigen::Vector3d::Zero()),
      max_translational_velocity(Eigen::Vector3d::Zero()),
      min_rotation_angle(Eigen::Vector3d::Constant(-M_PI)),
      max_rotation_angle(Eigen::Vector3d::Constant(M_PI)),
      reference_quaternion_for_min_max(Eigen::Quaterniond::Identity()) {};

// todo(johntgz) remove test method
CartesianLimits CartesianLimits::GenerateTestParams() {
  CartesianLimits cartesian_limits;

  cartesian_limits.min_translational_position =
      Eigen::Vector3d(-5.0, -5.0, -5.0);
  cartesian_limits.max_translational_position = Eigen::Vector3d(5.0, 5.0, 5.0);
  cartesian_limits.min_translational_velocity =
      Eigen::Vector3d(-5.0, -5.0, -5.0);
  cartesian_limits.max_translational_velocity = Eigen::Vector3d(5.0, 5.0, 5.0);

  return cartesian_limits;
}

}  // namespace aic_controller
