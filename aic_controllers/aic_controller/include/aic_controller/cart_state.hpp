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

#ifndef AIC_CONTROLLER__CART_STATE_HPP_
#define AIC_CONTROLLER__CART_STATE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace aic {

// Cartesian state with pose, velocity and acceleration
struct CartState {
  Eigen::Isometry3d pose;
  Eigen::Matrix<double, 6, 1> velocity;
  Eigen::Matrix<double, 6, 1> acceleration;

  /**
   * @brief Default constructor
   *
   */
  CartState()
      : pose(Eigen::Isometry3d::Identity()),
        velocity(Eigen::Matrix<double, 6, 1>::Zero()),
        acceleration(Eigen::Matrix<double, 6, 1>::Zero()) {};
};

}  // namespace aic

#endif  // AIC_CONTROLLER__CART_STATE_HPP_
