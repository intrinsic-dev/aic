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

#ifndef AIC_CONTROLLER__CARTESIAN_LIMITS_HPP_
#define AIC_CONTROLLER__CARTESIAN_LIMITS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace aic_controller {

//==============================================================================
// Cartesian limits with translational and rotation components
struct CartesianLimits {
  // Translational position and velocity limits
  Eigen::Vector3d min_translational_position;
  Eigen::Vector3d max_translational_position;
  Eigen::Vector3d min_translational_velocity;
  Eigen::Vector3d max_translational_velocity;

  // Euler angles rotation limits
  Eigen::Vector3d min_rotation_angle;
  Eigen::Vector3d max_rotation_angle;
  double max_rotational_velocity;
  // Reference quaternion for this min/max rotation.
  Eigen::Quaterniond reference_quaternion_for_min_max;

  /**
   * @brief Default constructor
   *
   */
  CartesianLimits();
};

}  // namespace aic_controller

#endif  // AIC_CONTROLLER__CARTESIAN_LIMITS_HPP_
