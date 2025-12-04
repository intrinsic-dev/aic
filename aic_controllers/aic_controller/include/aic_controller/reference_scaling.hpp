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

#ifndef AIC_CONTROLLER__REFERENCE_SCALING_HPP_
#define AIC_CONTROLLER__REFERENCE_SCALING_HPP_

#include <Eigen/Core>
#include <optional>

namespace aic {

// todo(johtngz) capture joint_positions, joint_velocities and
// joint_accelerations by value
//  OR we could simply pass in JointState and clamp everything
bool ClampJointStatesToLimits(
    const JointLimits& limits, Eigen::VectorXd& joint_positions,
    const double soft_margin_radians = 0.0,
    std::optional<Eigen::VectorXd&> joint_velocities = std::nullopt,
    std::optional<Eigen::VectorXd&> joint_accelerations = std::nullopt) {
  bool mutated = false;

  if (joint_velocities.has_value()) {
    mutated = ScaleJointVelocitiesToLimits(limits, joint_velocities.value());
  }

  if (joint_accelerations.has_value()) {
    mutated =
        ScaleJointAccelerationsToLimits(limits, joint_accelerations.value());
  }

  for (std::size_t k = 0; k < joint_positions->size(); ++k) {
    if (joint_positions[k] >= limits.max_position[k] - soft_margin_radians) {
      // Clamp only if position exceeds limits
      if (joint_positions[k] > limits.max_position[k]) {
        joint_positions[k] = limits.max_position[k];
        mutated = true;

        RCLCPP_WARN(get_node()->get_logger(),
                    "Maximum position limit violation: Reducing joint "
                    "position[%ld] to %f",
                    k, limits.max_position[k]);
      }

      // If velocity is positive, smoothly ramp velocity and acceleration down
      // to zero based on the proximity to the limit
      if (joint_velocities.has_value()) {
        if ((joint_velocities.value())[k] > 0.0) {
          double scale_factor = 0.0;
          if (soft_margin_radians > 0.0) {
            double distance_from_soft_margin =
                joint_positions[k] -
                (limits.max_position[k] - soft_margin_radians);
            // Normalized distance x from the soft margin start
            double x = std::clamp(
                distance_from_soft_margin / soft_margin_radians, 0.0, 1.0);
            scale_factor = (1. - x * x) * (1.0 - x * x);
          }
          (joint_velocities.value())[k] *= scale_factor;
          if (joint_accelerations.has_value()) {
            (joint_accelerations.value())[k] *= scale_factor;
          }
          mutated = true;
        }
      }

    } else if (joint_positions[k] <=
               limits.min_position[k] + soft_margin_radians) {
      // Clamp only if position exceeds limits
      if (joint_positions[k] < limits.min_position[k]) {
        joint_positions[k] = limits.min_position[k];
        mutated = true;

        RCLCPP_WARN(get_node()->get_logger(),
                    "Minimum position limit violation: Increasing joint "
                    "position[%ld] to %f",
                    k, limits.min_position[k]);
      }

      // If velocity is negative, smoothly ramp velocity and acceleration down
      // to zero based on the proximity to the limit
      if (joint_velocities.has_value()) {
        if ((joint_velocities.value())[k] < 0.0) {
          double scale_factor = 0.0;
          if (soft_margin_radians > 0.0) {
            double distance_from_soft_margin =
                (limits.min_position[k] + soft_margin_radians) -
                joint_positions[k];
            // Normalized distance x from the soft margin start
            double x = std::clamp(
                distance_from_soft_margin / soft_margin_radians, 0.0, 1.0);
            scale_factor = (1. - x * x) * (1.0 - x * x);
          }
          (joint_velocities.value())[k] *= scale_factor;
          if (joint_accelerations.has_value()) {
            (joint_accelerations.value())[k] *= scale_factor;
          }
          mutated = true;
        }
      }
    }
  }

  return mutated;
}

bool ScaleJointVelocitiesToLimits(const JointLimits& limits,
                                  Eigen::VectorXd& joint_velocities) {
  // Compute scaling factor
  double scaling_factor = 1.0;
  bool scaled = false;

  for (std::size_t k = 0; k < joint_velocities.size(); ++k) {
    double scaling_factor_candidate = 1.0;
    if (joint_velocities[k] > limits.max_velocity[k]) {
      scaling_factor_candidate = limits.max_velocity[k] / joint_velocities[k];
    } else if (joint_velocities[k] < -limits.max_velocity[k]) {
      scaling_factor_candidate = -limits.max_velocity[k] / joint_velocities[k];
    }
    if (scaling_factor_candidate < scaling_factor) {
      scaling_factor = scaling_factor_candidate;
    }
  }

  if (scaling_factor < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Negative scaling factor computed from scaling joint "
                 "velocities. Please fix invalid joint limits. Scaling "
                 "velocities to zero.");

    scaled = true;
    joint_velocities *= 0.0;
  } else if (scaling_factor < 1.0) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Velocity limit violation: Scaling joint velocity by %f",
                scaling_factor);

    scaled = true;
    joint_velocities *= scaling_factor;
  }

  return scaled;
}

bool ScaleJointAccelerationsToLimits(const JointLimits& limits,
                                     Eigen::VectorXd& joint_accelerations) {
  // Find scaling factor.
  double scaling_factor = 1.0;
  bool scaled = false;

  for (std::size_t k = 0; k < joint_accelerations.size(); ++k) {
    double scaling_factor_candidate = 1.0;
    if (joint_accelerations[k] > limits.max_acceleration[k]) {
      scaling_factor_candidate =
          limits.max_acceleration[k] / joint_accelerations[k];
    } else if (joint_accelerations[k] < -limits.max_acceleration[k]) {
      scaling_factor_candidate =
          -limits.max_acceleration[k] / joint_accelerations[k];
    }
    if (scaling_factor_candidate < scaling_factor) {
      scaling_factor = scaling_factor_candidate;
    }
  }

  if (scaling_factor < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Negative scaling factor computed from scaling joint "
                 "accelerations. Please fix invalid joint limits. Scaling "
                 "accelerations to zero.");

    scaled = true;
    joint_accelerations *= 0.0;
  } else if (scaling_factor < 1.0) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "Acceleration limit violation: Scaling joint acceleration by %f",
        scaling_factor);

    scaled = true;
    joint_accelerations *= scaling_factor;
  }

  return scaled;
}

}  // namespace aic

#endif  // AIC_CONTROLLER__REFERENCE_SCALING_HPP_
