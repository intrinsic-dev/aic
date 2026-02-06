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

#ifndef AIC_CONTROLLER__ACTIONS__CARTESIAN_IMPEDANCE_ACTION_HPP_
#define AIC_CONTROLLER__ACTIONS__CARTESIAN_IMPEDANCE_ACTION_HPP_

#include <Eigen/Core>

#include "aic_controller/cartesian_state.hpp"
#include "aic_controller/utils.hpp"
#include "joint_limits/joint_limits.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Interfaces
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

//==============================================================================
namespace aic_controller {
using trajectory_msgs::msg::JointTrajectoryPoint;

//==============================================================================
struct CartesianImpedanceParameters {
  // This is required for fixed-size Eigen types to precent segmentation faults
  // resulting from memory alignment issues
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Nullspace control parameters
  Eigen::VectorXd nullspace_goal;
  Eigen::VectorXd nullspace_stiffness;
  Eigen::VectorXd nullspace_damping;
  // Parameters for impedance control.
  Eigen::Matrix<double, 6, 6> stiffness_matrix;
  Eigen::Matrix<double, 6, 6> damping_matrix;
  Eigen::Matrix<double, 6, 1> pose_error_integrator_gain;
  Eigen::Matrix<double, 6, 1> pose_error_integrator_bound;
  Eigen::Matrix<double, 6, 1> maximum_wrench;
  Eigen::Matrix<double, 6, 1> feedforward_wrench;
  Eigen::Matrix<double, 6, 1> feedforward_interpolation_wrench_min;
  Eigen::Matrix<double, 6, 1> feedforward_interpolation_wrench_max;
  Eigen::Matrix<double, 6, 1> feedforward_interpolation_max_wrench_dot;
  // Offset added to control wrench to account for payload weight
  Eigen::Matrix<double, 6, 1> offset_wrench;
  // The activation_percentage specifies the percentage (0,1) of the range in
  // which the activation potential is active.
  // It is used as follows:
  // joint_min + ((( joint_max - joint_min)/ 2) * activation percentage)
  // and
  // joint_max - ((( joint_max - joint_min) / 2)  * activation percentage).
  // When the activation percentage is 1, the activation potential is active for
  // the whole range of joint positions.
  double activation_percentage;

  CartesianImpedanceParameters() : activation_percentage(0.0) {
    stiffness_matrix.setZero();
    damping_matrix.setZero();
    pose_error_integrator_gain.setZero();
    pose_error_integrator_bound.setZero();
    maximum_wrench.setConstant(std::numeric_limits<double>::infinity());
    feedforward_wrench.setZero();
    feedforward_interpolation_wrench_min.setZero();
    feedforward_interpolation_wrench_max.setZero();
    feedforward_interpolation_max_wrench_dot.setZero();
  }

  explicit CartesianImpedanceParameters(int num_joints)
      : nullspace_goal(Eigen::VectorXd::Zero(num_joints)),
        nullspace_stiffness(Eigen::VectorXd::Zero(num_joints)),
        nullspace_damping(Eigen::VectorXd::Zero(num_joints)),
        stiffness_matrix(Eigen::Matrix<double, 6, 6>::Zero()),
        damping_matrix(Eigen::Matrix<double, 6, 6>::Zero()),
        pose_error_integrator_gain(Eigen::Matrix<double, 6, 1>::Zero()),
        pose_error_integrator_bound(Eigen::Matrix<double, 6, 1>::Zero()),
        maximum_wrench(Eigen::Matrix<double, 6, 1>::Constant(
            std::numeric_limits<double>::infinity())),
        feedforward_wrench(Eigen::Matrix<double, 6, 1>::Zero()),
        feedforward_interpolation_wrench_min(
            Eigen::Matrix<double, 6, 1>::Zero()),
        feedforward_interpolation_wrench_max(
            Eigen::Matrix<double, 6, 1>::Zero()),
        feedforward_interpolation_max_wrench_dot(
            Eigen::Matrix<double, 6, 1>::Zero()) {}
};

//==============================================================================
class CartesianImpedanceAction {
 public:
  CartesianImpedanceAction(std::size_t num_joints);

  /**
   * @brief Configure the cartesian impedance action with joint limits and
   * relevant node interfaces
   *
   * @param joint_limits Joint limits on joint position, velocity, accelration
   * @param logging_if Node interface for logging
   * @return true
   * @return false
   */
  [[nodiscard]]
  bool configure(
      const std::vector<joint_limits::JointLimits>& joint_limits,
      const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr&
          logging_if,
      const rclcpp::node_interfaces::NodeClockInterface::SharedPtr& clock_if);

  /**
   * @brief Generates a target joint torque with the control torque given the
   * nullspace, tool goal, sensed joint position and velocity, and control
   * parameters.
   *
   * @param tool_pose_error Pose error between tool target and current state
   * @param tool_vel_error Velocity error between tool target and current state
   * @param current_joint_state Current state of joints
   * @param jacobian Jacobian of robot arm
   * @param impedance_params Impedance controller parameters and further
   * derivatives
   * @param new_joint_reference Joint target torque
   * @return true
   * @return false
   */
  bool compute(const Eigen::Matrix<double, 6, 1>& tool_pose_error,
               const Eigen::Matrix<double, 6, 1>& tool_vel_error,
               const JointTrajectoryPoint& current_joint_state,
               const Eigen::MatrixXd& jacobian,
               const CartesianImpedanceParameters& impedance_params,
               JointTrajectoryPoint& new_joint_reference);

 private:
  /**
   * @brief Returns the nullspace control torque for a desired
   * nullspace configuration.
   *
   * @param nullspace_stiffness Control stiffness for each joint
   * @param nullspace_damping Control damping for each joint
   * @param nullspace_goal Desired nullspace joint configurations
   * @param current_joint_state Current position of the joints
   * @param jacobian Current jacobian matrix of the robot arm
   * @param nullspace_stiffness_torque Output nullspace stiffness control torque
   * @param svd_threshold Used to calculate the condition_number_threshold,
   * which determines the threshold value of the maximum singular value
   * over other singular values before applying regularization.
   * Passed as input to the pseudoinverse function.
   * @return true Computation is successful
   * @return false Computation unsuccessful due to failed conditional checks
   */
  bool compute_nullspace_torque(const Eigen::VectorXd& nullspace_stiffness,
                                const Eigen::VectorXd& nullspace_damping,
                                const Eigen::VectorXd& nullspace_goal,
                                const JointTrajectoryPoint& current_joint_state,
                                const Eigen::MatrixXd& jacobian,
                                Eigen::VectorXd& nullspace_stiffness_torque,
                                double svd_threshold = 0.001);

  /**
   * @brief Computes the right pseudo-inverse of matrix M using SVD smoothed
   * regulatization. This largely follows the paper:
   *
   * Chiaverini, S. (1997). Singularity-robust task-priority redundancy
   * resolution for real-time kinematic control of robot manipulators. IEEE
   * Transactions on Robotics and Automation, 13(3), 398–410.
   * doi: 10.1109/70.585902
   *
   * The SVD of the weighted matrix inversion is:
   *
   *   M * M^T = U * S * V^T
   *
   * where U and V are orthonormal matrices, S is a diagonal matrix, and W is a
   * positive definite square weight matrix.
   *
   * The pseudo-inverse is computed as:
   *
   *   M^dagger = M^T * inv(U * S * V^T)
   *            = M^T * V * inv(S) * U^T
   *
   * For the smoothed regularized pseudo-inverse this becomes:
   *   M^dagger = M^T * V * S^dagger * U^T
   *
   * where S^dagger is the diagonal matrix with the entries:
   *
   *   S^dagger_{ii} = S_{ii} / (S_{ii}^2 + lambda_{i}^2)
   *
   *   lambda_{i}^2 = (1 - (condition_number_threshold / cn_{i})^2)^2 *
   *                  (1 / epsilon)  for cn_{i} > condition_number_threshold
   *   lambda_{i}^2 = 0              for cn_{i} <= condition_number_threshold
   *
   *   for i = 0, ..., m.cols()-1
   *
   * The epsilon regularizer is a small positive number to avoid numerical
   * issues, equivalent to ridge regression regularization.
   *
   * In contrast to the Tikhonov regularization, the lambda_{i} are adapted per
   * dimension to the condition number of this dimension, and regularization is
   * added in a smooth continuous/differentiable way. For robot control, this
   * ensures that regularization does not create discountinous jumps in the
   * control signals.
   *
   * The default values for `condition_number_threshold` and `epsilon` are set
   * to conservative values practical for the control of robot arms.
   *
   * @param M Matrix to compute right pseudo-inverse on
   * @param M_pinv Matrix with right pseudo-inverse
   * @param condition_number_threshold Determines the threshold value of the
   * maximum singular value over other singular values before applying
   * regularization.
   * @param epsilon Small tolerance value used in regularization
   * @return true Computation is successful
   * @return false Computation unsuccessful due to failed conditional checks
   */
  bool compute_smooth_right_pseudo_inverse(
      const Eigen::MatrixXd& M, Eigen::MatrixXd& M_pinv,
      double condition_number_threshold = 1500, double epsilon = 1e-6);

  /**
   * @brief Implementation of a quadratic potential field that computes the
   * torque required to move the joint state away from given joint limit.
   * When the joint state exceeds the activation threshold range
   * (lower_activation_threshold, upper_activation_threshold) but has not
   * exceeded the joint limits, the torque output is computed to move the joint
   * away from the limits.
   * If the joint states exeeds the joint limits, the saturated torque output at
   * minimum_absolute_torque or maximum_absolute_torque is used.
   *
   * @param lower_joint_state_limit Minimum joint limit
   * @param lower_activation_threshold Minimum threshold for activation of
   * avoidance torque
   * @param upper_joint_state_limit Maximum joint limit
   * @param upper_activation_threshold Maximum threshold for activation of
   * avoidance torque
   * @param minimum_absolute_torque Minimum torque magnitude
   * @param maximum_absolute_torque Maximum torque magnitude
   * @param joint_state Current joint position
   * @return double Joint avoidance torque that moves joint state away from
   * given joint limit.
   */
  double single_joint_avoidance_torque(double lower_joint_state_limit,
                                       double lower_activation_threshold,
                                       double upper_joint_state_limit,
                                       double upper_activation_threshold,
                                       double minimum_absolute_torque,
                                       double maximum_absolute_torque,
                                       double joint_state);

  // Number of robot joints
  const std::size_t num_joints_;
  std::vector<joint_limits::JointLimits> joint_limits_;

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_if_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_if_;

  Eigen::Matrix<double, 6, 1> pose_error_integrated_;
};

}  // namespace aic_controller

#endif  // AIC_CONTROLLER__ACTIONS__CARTESIAN_IMPEDANCE_ACTION_HPP_
