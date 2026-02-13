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
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include "aic_controller/cartesian_state.hpp"

// Interfaces
#include "geometry_msgs/msg/wrench.hpp"

//==============================================================================
namespace aic_controller {

//==============================================================================
namespace utils {

/**
 * @brief Computes the logarithmic map for SU(2), converting a Unit Quaternion
 * to it's corresponding tanget vector
 *
 * This is the inverse of the 'exp_map_quaternion' function
 *
 * @param quaternion A quaternion of unit length
 * @return Eigen::Vector3d Corresponding rotation vector in the tangent space of
 * SU(2)
 */
Eigen::Vector3d log_map_quaternion(const Eigen::Quaterniond& q);

/**
 * @brief Computes the Exponential Map for SU(2), converting a tangent vector to
 * a Unit Quaternion.
 *
 * This function implements the mapping from Lie Algebra to the Lie Group in
 * SU(2). Given a 3D tangent vector δ (representing an axis-angle rotation), it
 * calculates:
 *
 * R = exp(δ_hat)
 *
 * Where:
 * - δ (delta): A 3D vector in the tangent space (Lie Algebra).
 * - hat (^):   The operator mapping a 3D vector to a 2x2 skew-Hermitian matrix
 * - exp:       The matrix exponential function.
 *
 * Relationship:
 * The resulting quaternion represents a rotation of θ = ||δ|| radians around
 * the unit axis u = δ / ||δ||.
 *
 * @param delta A 3D tangent vector (Eigen::Vector3d) representing the rotation.
 * @return Eigen::Quaterniond The equivalent Unit Quaternion
 * (Eigen::Quaterniond)
 */
Eigen::Quaterniond exp_map_quaternion(const Eigen::Vector3d& delta);

/**
 * @brief Euler integration of a pose with the assumption of constant velocity
 * and zero acceleration
 *
 * @param pose Starting cartesian state
 * @param control_frequency Frequency of control loop in Hz
 * @return CartesianState Integrated cartesian state
 */
CartesianState integrate_pose(const CartesianState& pose,
                              const double& control_frequency);

/**
 * @brief Converts a ROS 2Wrench message to a Eigen 6x1 Matrix type
 *
 * @param msg The ROS 2 Wrench mesage to convert
 * @param wrench_eigen The converted 6x1 Eigen Matrix.
 */
void wrench_msg_to_eigen(const geometry_msgs::msg::Wrench& msg,
                         Eigen::Matrix<double, 6, 1>& wrench_eigen);

/**
 * @brief Converts a Eigen 6x1 Matrix type to ROS 2 Wrench message
 *
 * @param wrench_eigen The 6x1 Eigen Matrix to convert
 * @param msg The converted ROS 2 Wrench mesage
 */
void eigen_to_wrench_msg(const Eigen::Matrix<double, 6, 1>& wrench_eigen,
                         geometry_msgs::msg::Wrench& msg);

}  // namespace utils

}  // namespace aic_controller

#endif  // AIC_CONTROLLER__UTILS_HPP_
