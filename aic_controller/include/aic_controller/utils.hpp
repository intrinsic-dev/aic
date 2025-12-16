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

#ifndef AIC_CONTROLLER__UTILS_HPP_
#define AIC_CONTROLLER__UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "aic_controller/cartesian_state.hpp"

namespace aic_controller {

namespace utils {

/**
 * @brief Logarithmic map of the unit quaternion.
 *
 * This is the inverse of the 'expMapQuaternion' function
 *
 * @param quaternion quaternion q of unit length
 * @return Eigen::Vector3d Corresponding vector in the tangent space of SU(2)
 */
Eigen::Vector3d logMapQuaternion(const Eigen::Quaterniond& q);

/**
 * @brief Exponential map to the unit quaternion, a member of the SU(2) group.
 *
 * Let \f$ \exp \f$ be the matrix exponential and \f$ \hat{\cdot} \f$ be the
 * function which maps a tangent vector in SU(2) to its corresponding (2x2)
 * matrix representation. It holds that:
 *
 * \f$ \exp_{SU(2)}(\delta) = exp(\hat{delta}) \f$
 *
 * @param delta Tangent vector of SU(2)
 * @return Eigen::Quaterniond unit quaternion, a member of SU(2)
 */
Eigen::Quaterniond expMapQuaternion(const Eigen::Vector3d& delta);

/**
 * @brief Euler integration of a pose with the assumption of constant velocity
 * and zero acceleration
 *
 * @param pose Starting cartesian state
 * @param control_frequency Frequency of control loop in Hz
 * @return CartesianState Integrated cartesian state
 */
CartesianState IntegratePose(const CartesianState& pose,
                             const double& control_frequency);

}  // namespace utils

}  // namespace aic_controller

#endif  // AIC_CONTROLLER__UTILS_HPP_
