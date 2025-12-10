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

namespace aic_controller {

namespace utils {

Eigen::Vector3d expQuaternion(const Eigen::Quaterniond& quaternion) {
  // UNIMPLEMENTED
  // todo(johntgz)
  return Eigen::Vector3d::Zero();
}

Eigen::Quaterniond expQuaternion(const Eigen::Vector3d& delta) {
  // UNIMPLEMENTED
  // todo(johntgz)

  return Eigen::Quaterniond::Identity();
}

bool clamp_to_limits(const CartesianLimits& limits, const uint8_t& mode,
                     CartesianState& target_state, double soft_margin_meters,
                     double soft_margin_radians) {
  bool mutated = false;

  bool clamp_pose =
      mode == TrajectoryGenerationMode::MODE_POSITION ||
      mode == TrajectoryGenerationMode::MODE_POSITION_AND_VELOCITY;

  bool scale_velocity =
      mode == TrajectoryGenerationMode::MODE_VELOCITY ||
      mode == TrajectoryGenerationMode::MODE_POSITION_AND_VELOCITY;

  Eigen::Vector3d new_translation = target_state.pose.translation();
  Eigen::Matrix<double, 6, 1> new_velocity = target_state.velocity;

  // Scale linear and angular velocity
  if (scale_velocity) {
    // Compute scaling factor for translational velocity
    double translational_scaling_factor = 1.0;
    for (int k = 0; k < 3; ++k) {
      double translational_scaling_factor_candidate = 1.0;
      if (new_velocity(k) > limits.max_translational_velocity(k)) {
        translational_scaling_factor_candidate =
            limits.max_translational_velocity(k) / new_velocity(k);
      } else if (new_velocity(k) < limits.min_translational_velocity(k)) {
        translational_scaling_factor_candidate =
            limits.min_translational_velocity(k) / new_velocity(k);
      }
      if (translational_scaling_factor_candidate <
          translational_scaling_factor) {
        translational_scaling_factor = translational_scaling_factor_candidate;
      }
    }

    // Apply scaling factor for translational velocity
    if (translational_scaling_factor < 0.0) {
      RCLCPP_ERROR(this->get_node()->get_logger(),
                   "Encountered negative scaling factor while adjusting the "
                   "translational velocity. Ensure that the limit interval is "
                   "valid. Defaulting twist to zero.");

      new_velocity.array() *= 0.0;
      mutated = true;

    } else if (translational_scaling_factor < 1.0) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                           1000, "Scaling translational velocity by %f",
                           translational_scaling_factor);

      new_velocity.head(3) *= translational_scaling_factor;
      mutated = true;
    }

    // Find scaling factor for rotational velocity and apply it.
    if (new_velocity.tail(3).norm() > limits.max_rotational_velocity) {
      double rotational_scaling_factor =
          limits.max_rotational_velocity / new_velocity.tail(3).norm();
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                           1000, "Scaling rotational velocity by %f",
                           rotational_scaling_factor);
      new_velocity.tail(3) *= rotational_scaling_factor;
      mutated = true;
    }
  }

  if (clamp_pose) {
    // Clamp translational components to limits

    // Get mask for elements that exceeded soft margins
    auto trans_exceed_max_margin_mask =
        (new_translation.array() >=
         (limits.max_translational_position.array() - soft_margin_meters));
    auto trans_exceed_min_margin_mask =
        (new_translation.array() <=
         (limits.min_translational_position.array() + soft_margin_meters));

    new_translation =
        new_translation.cwiseMin(limits.max_translational_position);
    new_translation =
        new_translation.cwiseMax(limits.min_translational_position);

    // If translational soft margins are violated, smoothly scale linear
    // velocity to zero based on the distance to the limits
    if (scale_velocity) {
      for (std::size_t i = 0; i < 3; ++i) {
        if (!trans_exceed_max_margin_mask(i) &&
            !trans_exceed_min_margin_mask(i)) {
          continue;
        }
        double scale_factor = 0.0;
        if (soft_margin_meters > 0.0) {
          // Compute the normalized distance relative to the start of the soft
          // margin
          double distance_from_soft_margin = 0.0;
          if (new_velocity(i) > 0.0) {
            distance_from_soft_margin =
                new_translation(i) -
                (limits.max_translational_position(i) - soft_margin_meters);

          } else if (new_velocity(i) < 0.0) {
            distance_from_soft_margin =
                (limits.min_translational_position(i) + soft_margin_meters) -
                new_translation(i);
          }

          double normalized_distance_from_soft_margin = std::clamp(
              distance_from_soft_margin / soft_margin_meters, 0.0, 1.0);
          // The bi-square function creates a smooth and differentiable
          // transition between 0 and 1.
          scale_factor = (1. - normalized_distance_from_soft_margin *
                                   normalized_distance_from_soft_margin) *
                         (1.0 - normalized_distance_from_soft_margin *
                                    normalized_distance_from_soft_margin);
        }

        new_velocity(i) *= scale_factor;
      }
    }

    // Clamp rotational components to limits
    Eigen::Quaterniond new_quaternion(target_state.pose.rotation());

    // compute relative quaternion between current and reference quaternion
    Eigen::Quaterniond relative_quaternion =
        new_quaternion * limits.reference_quaternion_for_min_max.inverse();

    // The exponential log of the quaternion provides 0.5 * rotational_offset
    // from the reference quaternion. The 0.5 comes from the fact that
    // quaternions use rotation angles divided by 2 in sin() and cos()
    // functions. Thus the result of the logQuaternion needs to be multiplied
    // by 2.0.
    Eigen::Vector3d rotational_offset =
        2.0 * utils::logQuaternion(relative_quaternion);
    Eigen::Vector3d new_rotational_offset = rotational_offset;

    // Get mask for elements that exceeded soft margins
    auto rot_exceed_max_margin_mask =
        new_rotational_offset.array() >=
        (limits.max_rotation_angle.array() - soft_margin_radians);
    auto rot_exceed_min_margin_mask =
        new_rotational_offset.array() <=
        (limits.min_rotation_angle.array() + soft_margin_radians);

    // Clamp rotational offsets
    new_rotational_offset =
        new_rotational_offset.cwiseMax(limits.min_rotation_angle);
    new_rotational_offset =
        new_rotational_offset.cwiseMin(limits.max_rotation_angle);

    // If rotational soft margins are violated, smoothly scale angular
    // velocity to zero based on the distance to the limits
    if (scale_velocity) {
      for (std::size_t i = 0; i < 3; i++) {
        if (!rot_exceed_max_margin_mask(i) && !rot_exceed_min_margin_mask(i)) {
          continue;
        }
        double scale_factor = 0.0;
        if (soft_margin_radians > 0.0) {
          // Compute the normalized distance relative to the start of the soft
          // margin
          double distance_from_soft_margin = 0.0;
          if (new_velocity(i + 3) > 0.0) {
            distance_from_soft_margin =
                new_rotational_offset(i) -
                (limits.max_rotation_angle(i) - soft_margin_radians);

          } else if (new_velocity(i + 3) < 0.0) {
            distance_from_soft_margin =
                (limits.min_rotation_angle(i) + soft_margin_radians) -
                new_rotational_offset(i);
          }

          double normalized_distance_from_soft_margin = std::clamp(
              distance_from_soft_margin / soft_margin_radians, 0.0, 1.0);
          // The bi-square function creates a smooth and differentiable
          // transition between 0 and 1.
          scale_factor = (1. - normalized_distance_from_soft_margin *
                                   normalized_distance_from_soft_margin) *
                         (1.0 - normalized_distance_from_soft_margin *
                                    normalized_distance_from_soft_margin);
        }
        new_velocity(i + 3) *= scale_factor;
      }
    }

    // todo(johtngz)
    // Update pose quaternion with the modified rotational offset.
    if (!new_rotational_offset.isApprox(new_rotational_offset)) {
      RCLCPP_WARN_STREAM_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Limit violation: Rotational offset clamped to "
              << new_rotational_offset);

      // Divide the rotational offset by 2.0 to apply expQuaternion
      // correctly.
      new_rotational_offset.array() /= 2.0;
      target_state.pose.linear() =
          (utils::expQuaternion(new_rotational_offset) *
           limits.reference_quaternion_for_min_max)
              .toRotationMatrix();
      mutated = true;
    }

    if (!target_state.pose.translation().isApprox(new_translation)) {
      RCLCPP_WARN_STREAM_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Limit violation: Pose translation clamped to " << new_translation);

      target_state.pose.translation() = new_translation;
      mutated = true;
    }
    if (!target_state.velocity.isApprox(new_velocity)) {
      RCLCPP_WARN_STREAM_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Limit violation: Pose velocity clampted to " << new_velocity);

      target_state.velocity = new_velocity;
      mutated = true;
    }
  }

  return mutated;
}

}  // namespace utils

}  // namespace aic_controller
