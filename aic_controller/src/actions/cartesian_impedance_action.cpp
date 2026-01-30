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

#include "aic_controller/actions/cartesian_impedance_action.hpp"

namespace aic_controller {

//==============================================================================
CartesianImpedanceAction::CartesianImpedanceAction(std::size_t num_joints)
    : num_joints_(num_joints),
      pose_error_integrated_(Eigen::Matrix<double, 6, 1>::Zero()) {}

//==============================================================================
bool CartesianImpedanceAction::configure(
    const std::vector<joint_limits::JointLimits>& joint_limits,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_if,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr& clock_if) {
  joint_limits_ = joint_limits;
  logging_if_ = logging_if;
  clock_if_ = clock_if;

  return true;
}

//==============================================================================
bool CartesianImpedanceAction::compute(
    const Eigen::Matrix<double, 6, 1>& tool_pose_error,
    const Eigen::Matrix<double, 6, 1>& tool_vel_error,
    const JointTrajectoryPoint& current_joint_state,
    const Eigen::MatrixXd& jacobian, const CartesianImpedanceParameters& params,
    JointTrajectoryPoint& new_joint_reference) {
  // Compute the wrench using the equation
  // control_wrench = K * (x_des - x) + D * (v_des - v) + w_f
  // where D is damping, K is stiffness and w_f the feedforward wrench
  Eigen::Matrix<double, 6, 1> control_wrench =
      params.stiffness_matrix * tool_pose_error;
  control_wrench += params.damping_matrix * tool_vel_error;
  control_wrench += params.feedforward_wrench;

  // Integral term
  if ((params.pose_error_integrator_gain.array() < 0.0).any()) {
    RCLCPP_ERROR_STREAM(logging_if_->get_logger(),
                        "Invalid pose error integrator gain. "
                        "Required: >= 0. Received: "
                            << params.pose_error_integrator_gain);
    return false;
  }
  if ((params.pose_error_integrator_bound.array() < 0.0).any()) {
    RCLCPP_ERROR_STREAM(logging_if_->get_logger(),
                        "Invalid pose error integrator bound. "
                        "Required: >= 0. Received: "
                            << params.pose_error_integrator_bound);
    return false;
  }

  if ((params.pose_error_integrator_gain.array() > 0).any()) {
    pose_error_integrated_ += tool_pose_error;
    // Clamp to pose error integrated bounds
    pose_error_integrated_ =
        pose_error_integrated_.cwiseMin(params.pose_error_integrator_bound)
            .cwiseMax(-params.pose_error_integrator_bound);

    control_wrench +=
        params.pose_error_integrator_gain.cwiseProduct(pose_error_integrated_);
  }

  // Clamp to wrench value bounds
  control_wrench = control_wrench.cwiseMin(params.maximum_wrench)
                       .cwiseMax(-params.maximum_wrench);

  // Get target torque from jacobian
  Eigen::VectorXd target_torque = jacobian.transpose() * control_wrench;

  // Auxiliary controller using nullspace projection
  Eigen::VectorXd nullspace_stiffness_torque;
  if (!compute_nullspace_torque(params.nullspace_stiffness,
                                params.nullspace_damping, params.nullspace_goal,
                                current_joint_state, jacobian,
                                nullspace_stiffness_torque)) {
    RCLCPP_ERROR(logging_if_->get_logger(),
                 "Failed to compute nullspace stiffness torque.");
    return false;
  }
  target_torque += nullspace_stiffness_torque;

  //  Setting the activation thresholds as a percentage of joint range
  //  above/below the min/max joint limit guarantees correct behavior
  //  regardless of the joint limit signs. Negative (zero inclusive)
  //  parameter value is interpreted as deactivation of joint limit
  //  avoidance.
  if (params.activation_percentage > 0) {
    Eigen::VectorXd joint_limit_avoidance_torque(num_joints_);
    for (std::size_t k = 0; k < num_joints_; ++k) {
      double joint_range =
          joint_limits_[k].max_position - joint_limits_[k].min_position;
      double lower_activation_threshold =
          joint_limits_[k].max_position -
          (params.activation_percentage * (joint_range / 2));
      double upper_activation_threshold =
          joint_limits_[k].min_position +
          (params.activation_percentage * (joint_range / 2));

      if (joint_limits_[k].min_position >= lower_activation_threshold) {
        RCLCPP_ERROR(logging_if_->get_logger(),
                     "Lower Joint limit [%f] is <= lower activation threshold"
                     "[%f], which is invalid",
                     joint_limits_[k].min_position, lower_activation_threshold);
      }

      if (joint_limits_[k].max_position <= upper_activation_threshold) {
        RCLCPP_ERROR(logging_if_->get_logger(),
                     "Upper Joint limit [%f] is >= upper activation threshold"
                     "[%f], which is invalid",
                     joint_limits_[k].max_position, upper_activation_threshold);
      }

      joint_limit_avoidance_torque(k) = single_joint_avoidance_torque(
          joint_limits_[k].min_position, lower_activation_threshold,
          joint_limits_[k].max_position, upper_activation_threshold,
          joint_limits_[k].max_effort, joint_limits_[k].max_effort,
          current_joint_state.positions[k]);
    }

    target_torque += joint_limit_avoidance_torque;
  }

  // Clamp to joint torque limits
  for (std::size_t k = 0; k < num_joints_; ++k) {
    target_torque(k) =
        std::clamp(target_torque(k), -joint_limits_[k].max_effort,
                   joint_limits_[k].max_effort);
  }

  new_joint_reference.effort.resize(num_joints_);
  Eigen::VectorXd::Map(new_joint_reference.effort.data(),
                       new_joint_reference.effort.size()) = target_torque;

  return true;
}

//==============================================================================
bool CartesianImpedanceAction::compute_nullspace_torque(
    const Eigen::VectorXd& nullspace_stiffness,
    const Eigen::VectorXd& nullspace_damping,
    const Eigen::VectorXd& nullspace_goal,
    const JointTrajectoryPoint& current_joint_state,
    const Eigen::MatrixXd& jacobian,
    Eigen::VectorXd& nullspace_stiffness_torque, double svd_threshold) {
  if (std::size_t(nullspace_goal.rows()) != num_joints_) {
    RCLCPP_ERROR(logging_if_->get_logger(),
                 "num_joints_ and nullspace_goal size mismatch. Expected: %ld, "
                 "Actual: %ld",
                 num_joints_, nullspace_goal.size());
    return false;
  }

  if (current_joint_state.positions.size() != num_joints_) {
    RCLCPP_ERROR(
        logging_if_->get_logger(),
        "num_joints_ and current_joint_state size mismatch. Expected: %ld, "
        "Actual: %ld",
        num_joints_, current_joint_state.positions.size());
    return false;
  }

  if (std::size_t(jacobian.rows()) != 6) {
    RCLCPP_ERROR(logging_if_->get_logger(),
                 "jacobian.rows() != 6. Expected: 6, Actual: %ld",
                 jacobian.rows());
    return false;
  }

  if (std::size_t(jacobian.cols()) != num_joints_) {
    RCLCPP_ERROR(logging_if_->get_logger(),
                 "jacobian.cols and num_joints_ size mismatch. Expected: %ld, "
                 "Actual: %ld",
                 num_joints_, jacobian.cols());
    return false;
  }

  Eigen::VectorXd joint_positions = Eigen::Map<const Eigen::VectorXd>(
      current_joint_state.positions.data(),
      static_cast<Eigen::Index>(current_joint_state.positions.size()));
  Eigen::VectorXd joint_velocities = Eigen::Map<const Eigen::VectorXd>(
      current_joint_state.velocities.data(),
      static_cast<Eigen::Index>(current_joint_state.velocities.size()));

  // Compute the control torque resulting from the nullspace stiffness.
  Eigen::VectorXd nullspace_torque =
      nullspace_stiffness.cwiseProduct(nullspace_goal - joint_positions);
  // Compute the nullspace damping.
  nullspace_torque -= nullspace_damping.cwiseProduct(joint_velocities);

  double condition_number_threshold = 1500;
  if (svd_threshold > 0) {
    condition_number_threshold = 1.0 / svd_threshold;
  }

  Eigen::MatrixXd jacobian_pinv;
  if (!compute_smooth_right_pseudo_inverse(jacobian, jacobian_pinv,
                                           condition_number_threshold)) {
    RCLCPP_ERROR(logging_if_->get_logger(),
                 "Failed to compute right pseudo-inverse of the Jacobian.");
    return false;
  }

  Eigen::MatrixXd nullspace_projector =
      Eigen::MatrixXd::Identity(num_joints_, num_joints_) -
      (jacobian * jacobian_pinv);
  nullspace_stiffness_torque = nullspace_projector * nullspace_torque;

  return true;
}

//==============================================================================
bool CartesianImpedanceAction::compute_smooth_right_pseudo_inverse(
    const Eigen::MatrixXd& M, Eigen::MatrixXd& M_pinv,
    double condition_number_threshold, double epsilon) {
  if (M.cols() < M.rows()) {
    RCLCPP_ERROR(logging_if_->get_logger(),
                 "Matrix M must be square or have more columns than rows.");
    return false;
  }

  if (condition_number_threshold < 0.0) {
    RCLCPP_ERROR(logging_if_->get_logger(),
                 "Condition number threshold must be >= 0.");
    return false;
  }

  if (epsilon <= 0.0) {
    RCLCPP_ERROR(logging_if_->get_logger(),
                 "epislon (for regularization) must be > 0.");
    return false;
  }

  // Compute the SVD of M * M^T.
  Eigen::MatrixXd M_MT = M * M.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      M_MT, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::VectorXd singular_values = svd.singularValues();
  const double max_singular_value = singular_values(0);
  if (std::abs(max_singular_value) <= std::numeric_limits<double>::epsilon()) {
    RCLCPP_ERROR(logging_if_->get_logger(),
                 "Matrix is all-zero, cannot compute pseudo-inversion.");
    return false;
  }

  // Compute regularized inverse singular value matrix.
  Eigen::MatrixXd S(M_MT.rows(), M_MT.cols());
  S.setZero();
  for (std::size_t i = 0; i < std::size_t(singular_values.size()); ++i) {
    double cn_i = max_singular_value / (singular_values(i) + 1e-10);
    double lambda_i_squared = 0.0;

    if (cn_i > condition_number_threshold) {
      double ratio = condition_number_threshold / cn_i;
      // This is the bisquare smoothing function which has zero derivatives
      // at ratio=0 and ratio=1 and thus a smooth onset and offset of the
      // regularization.
      lambda_i_squared =
          (1 - ratio * ratio) * (1 - ratio * ratio) * (1.0 / epsilon);
    }
    S(i, i) = (singular_values(i)) /
              (singular_values(i) * singular_values(i) + lambda_i_squared);
  }

  M_pinv = Eigen::MatrixXd(M.transpose() * svd.matrixV() * S *
                           svd.matrixU().transpose());

  return true;
}

//==============================================================================
double CartesianImpedanceAction::single_joint_avoidance_torque(
    double lower_joint_state_limit, double lower_activation_threshold,
    double upper_joint_state_limit, double upper_activation_threshold,
    double minimum_absolute_torque, double maximum_absolute_torque,
    double joint_state) {
  double torque = 0.0;
  // upper_gain and lower_gain are used to reach the maximum_absolute_torque at
  // the joint limit
  double upper_gain =
      std::abs(minimum_absolute_torque /
               std::abs(upper_joint_state_limit - upper_activation_threshold));
  double lower_gain =
      maximum_absolute_torque /
      std::abs(lower_joint_state_limit - lower_activation_threshold);

  if (joint_state > upper_joint_state_limit) {
    // Current joint state exceeded upper joint limit, saturate the torque
    // output.
    RCLCPP_ERROR_THROTTLE(
        logging_if_->get_logger(), *clock_if_->get_clock(), 1000,
        "Joint limit avoidance potential saturated at upper limit!");

    torque =
        upper_gain * (upper_activation_threshold - upper_joint_state_limit);

  } else if (joint_state > upper_activation_threshold) {
    // activation_threshold < joint_state < joint_limit
    // Current joint state is in the active zone. Compute torque according to
    // joint_state.
    RCLCPP_ERROR_THROTTLE(
        logging_if_->get_logger(), *clock_if_->get_clock(), 1000,
        "Joint limit avoidance potential is active at upper limit!");

    torque = upper_gain * (upper_activation_threshold - joint_state);

  } else if (joint_state < lower_joint_state_limit) {
    // Current joint state exceeded lower joint limit, saturate the torque
    // output.
    RCLCPP_ERROR_THROTTLE(
        logging_if_->get_logger(), *clock_if_->get_clock(), 1000,
        "Joint limit avoidance potential saturated at lower limit!");

    torque =
        lower_gain * (lower_activation_threshold - lower_joint_state_limit);

  } else if (joint_state < lower_activation_threshold) {
    // activation_threshold < joint_state < joint_limit
    // Current joint state in the active zone. Compute torque according to
    // joint_state.
    RCLCPP_ERROR_THROTTLE(
        logging_if_->get_logger(), *clock_if_->get_clock(), 1000,
        "Joint limit avoidance potential active at lower limit!");

    torque = lower_gain * (lower_activation_threshold - joint_state);

  } else {
    // joint_state < activation_threshold
    // Current joint state is outside the active zone. No torque output required
    torque = 0.0;
  }

  return torque;
}

}  // namespace aic_controller
