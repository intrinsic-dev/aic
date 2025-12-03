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

#ifndef AIC_CONTROLLER__JOINT_STATE_HPP_
#define AIC_CONTROLLER__JOINT_STATE_HPP_

#include <Eigen/Core>

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace aic_controller {

// Joint states for position, velocity and acceleration
struct JointState {
  Eigen::VectorXd positions;
  Eigen::VectorXd velocities;
  Eigen::VectorXd accelerations;

  /**
   * @brief Default constructor
   *
   */
  JointState() = default;

  /**
   * @brief Construct JointState from JointTrajectoryPoint ROS message
   *
   * @param msg JointTrajectoryPoint message to be constructed from
   */
  explicit JointState(const trajectory_msgs::msg::JointTrajectoryPoint& msg) {
    positions = Eigen::Map<const Eigen::VectorXd>(msg.positions.data(),
                                                  msg.positions.size());

    std::size_t size = msg.positions.size();

    // Validate size of joint velocities and accelerations, if they don't match
    // the size of the joint positions, then set them to zero.
    if (msg.velocities.size() == size) {
      velocities = Eigen::Map<const Eigen::VectorXd>(msg.velocities.data(),
                                                     msg.velocities.size());
    } else {
      velocities = Eigen::VectorXd::Zero(size);
    }

    if (msg.accelerations.size() == size) {
      accelerations = Eigen::Map<const Eigen::VectorXd>(
          msg.accelerations.data(), msg.accelerations.size());
    } else {
      accelerations = Eigen::VectorXd::Zero(size);
    }
  }

  /**
   * @brief Convert from JointState to ROS message of type JointTrajectoryPoint
   *
   * @return trajectory_msgs::msg::JointTrajectoryPoint
   */
  trajectory_msgs::msg::JointTrajectoryPoint to_msg() const {
    trajectory_msgs::msg::JointTrajectoryPoint msg;

    std::size_t size = positions.size();

    msg.positions.resize(size);
    Eigen::VectorXd::Map(&msg.positions[0], size) = positions;

    if (velocities.size() == size) {
      msg.velocities.resize(size);
      Eigen::VectorXd::Map(&msg.velocities[0], size) = velocities;
    } else {
      msg.velocities = std::vector<double>(size);
    }

    if (accelerations.size() == size) {
      msg.accelerations.resize(size);
      Eigen::VectorXd::Map(&msg.accelerations[0], size) = accelerations;
    } else {
      msg.accelerations = std::vector<double>(size);
    }

    return msg;
  }
};

}  // namespace aic_controller

#endif  // AIC_CONTROLLER__JOINT_STATE_HPP_
