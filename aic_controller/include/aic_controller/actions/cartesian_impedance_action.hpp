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
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
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
  Eigen::Matrix<double, 6, 6> mass_matrix;
  Eigen::Matrix<double, 6, 1> pose_error_integrator_gain;
  Eigen::Matrix<double, 6, 1> pose_error_integrator_bound;
  Eigen::VectorXd joint_torque_limits;
  Eigen::Matrix<double, 6, 1> feedforward_wrench;
  Eigen::Matrix<double, 6, 1> maximum_wrench;
  // The activation_percentage specifies the percentage (0,1) of the range in
  // which the activation potential is active.
  // It is used as follows:
  // joint_min + ((( joint_max - joint_min)/ 2) * activation percentage)
  // and
  // joint_max - ((( joint_max - joint_min) / 2)  * activation percentage).
  // When the activation percentage is 1, the activation potential is active for
  // the whole range of joint positions.
  double activation_percentage;

  // todo(johntgz) do we need default constructor?
  CartesianImpedanceParameters() : activation_percentage(0.0) {
    stiffness_matrix.setZero();
    damping_matrix.setZero();
    mass_matrix.setZero();
    pose_error_integrator_gain.setZero();
    pose_error_integrator_bound.setZero();
    feedforward_wrench.setZero();

    maximum_wrench.setConstant(std::numeric_limits<double>::infinity());
  }

  explicit CartesianImpedanceParameters(int num_joints)
      : nullspace_goal(Eigen::VectorXd::Zero(num_joints)),
        nullspace_stiffness(Eigen::VectorXd::Zero(num_joints)),
        nullspace_damping(Eigen::VectorXd::Zero(num_joints)),
        stiffness_matrix(Eigen::Matrix<double, 6, 6>::Zero()),
        damping_matrix(Eigen::Matrix<double, 6, 6>::Zero()),
        mass_matrix(Eigen::Matrix<double, 6, 6>::Zero()),
        pose_error_integrator_gain(Eigen::Matrix<double, 6, 1>::Zero()),
        pose_error_integrator_bound(Eigen::Matrix<double, 6, 1>::Zero()),
        joint_torque_limits(Eigen::VectorXd::Zero(num_joints)),
        feedforward_wrench(Eigen::Matrix<double, 6, 1>::Zero()),
        maximum_wrench(Eigen::Matrix<double, 6, 1>::Constant(
            std::numeric_limits<double>::infinity())) {}
};

//==============================================================================
class CartesianImpedanceAction {
 public:
  CartesianImpedanceAction(std::size_t num_joints);

  /**
   * @brief todo(johntgz)
   *
   * @param joint_limits Joint limits on joint position, velocity, accelration
   * @param logging_if Node interface for logging
   * @return true
   * @return false
   */
  [[nodiscard]]
  bool Configure(
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
  bool Compute(const Eigen::Matrix<double, 6, 1>& tool_pose_error,
               const Eigen::Matrix<double, 6, 1>& tool_vel_error,
               const JointTrajectoryPoint& current_joint_state,
               const Eigen::MatrixXd& jacobian,
               const CartesianImpedanceParameters& impedance_params,
               JointTrajectoryPoint& new_joint_reference);

 private:
  // todo(johntgz) docstrings

  // Returns the nullspace stiffness control torque for a desired nullspace
  // configuration given a measured joint configuration with joint space
  // damping. Real time safe with a static N_DOF. Params nullspace_stiffness
  //   the control stiffness for each joint
  // nullspace_joint_position_goal
  //   the desired nullsapce joint configuration
  // sensed_joint_position
  //   the current sensed position of the joints
  // jacobian
  //   the current jacobian of the robot
  // svd_threshold
  //   the threshold percentage over the maximum singular value used to cancel
  //   small singular values. Passed as input to the pseudoinverse function
  bool ComputeNullspaceStiffnessTorque(
      const Eigen::VectorXd& nullspace_stiffness,
      const Eigen::VectorXd& nullspace_damping,
      const Eigen::VectorXd& nullspace_goal,
      const JointTrajectoryPoint& current_joint_state,
      const Eigen::MatrixXd& jacobian,
      Eigen::VectorXd& nullspace_stiffness_torque,
      double svd_threshold = 0.001);

  // todo(johntgz) docstrings

  // Computes matrix right pseudo-inverse with SVD smoothed regularization,
  // including an optional weight matrix 'W'. This follows largely the paper:
  //
  // Chiaverini, S. (1997). Singularity-robust task-priority redundancy
  // resolution for real-time kinematic control of robot manipulators. IEEE
  // Transactions on Robotics and Automation, 13(3), 398–410.
  // doi: 10.1109/70.585902
  //
  // The SVD of the weighted matrix inversion is:
  //
  //  M * W * M^T = U * S * V^T
  //
  // where U and V are orthonormal matrices, S is a diagonal matrix, and W is a
  // positive definite square weight matrix.
  //
  // The pseudo-inverse is computed as
  //
  //  m^dagger = W * M^T * inv(U * S * V^T)
  //           = W * M^T * V * inv(S) * U^T
  //
  // For the smoothed regularized pseudo-inverse this becomes:
  //  m^dagger = W * M^T * V * S^dagger * U^T
  //
  // where S^dagger is the diagonal matrix with entries
  //
  //  S^dagger_{ii} = S_{ii} / (S_{ii}^2 + lambda_{i}^2)
  //
  //  cn_{i} = S_{0} / S_{ii}
  //
  //  lambda_{i}^2 = (1 - (condition_number_threshold / cn_{i})^2)^2 *
  //                 (1 / epsilon)  for cn_{i} > condition_number_threshold
  //
  //  lambda_{i}^2 = 0              for cn_{i} <= condition_number_threshold
  //
  // for i = 0, ..., m.cols()-1.
  //
  // The epsilon regularizer is a small positive number to avoid numerical
  // issues, equivalent to a ridge regression regularization.
  //
  // In contrast to the Tikhonov regularization, the lambda_{i} are adapted per
  // dimension to the condition number of this dimension, and regularization is
  // added in a smooth continuous/differentiable way. For robot control, this
  // ensures that regularization does not create discountinous jumps in the
  // control signals.
  //
  // Returns an error status if M has all zero singular values or matrix M and
  // weight matrix W have incompatible dimensions or incorrect input arguments.
  //
  // Default values for `epsilon` and `condition_number_threshold` are set to
  // conservative values as useful for robot control.
  //

  /**
   * @brief
   *
   * @param M Matrix to compute right pseudo-inverse on
   * @param M_pinv Matrix with right pseudo-inverse
   * @param condition_number_threshold
   * @param epsilon
   * @return true
   * @return false
   */
  bool ComputeSmoothRightPseudoInverse(const Eigen::MatrixXd& M,
                                       Eigen::MatrixXd& M_pinv,
                                       double condition_number_threshold = 1500,
                                       double epsilon = 1e-6);

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
   * @param lower_joint_state_limit
   * @param lower_activation_threshold
   * @param upper_joint_state_limit
   * @param upper_activation_threshold
   * @param minimum_absolute_torque
   * @param maximum_absolute_torque
   * @param joint_state
   * @return double
   */
  double SingleJointAvoidanceTorque(double lower_joint_state_limit,
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
