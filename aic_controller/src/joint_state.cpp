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

#include "aic_controller/joint_state.hpp"

namespace aic_controller {

//==============================================================================
JointState::JointState() : num_joints_(0) {}

//==============================================================================
JointState::JointState(const std::size_t num_joints)
    : num_joints_(num_joints),
      positions(Eigen::VectorXd::Zero(num_joints)),
      velocities(Eigen::VectorXd::Zero(num_joints)) {}

//==============================================================================
//==============================================================================
JointState::JointState(JointTrajectoryPoint& msg, const std::size_t num_joints)
    : num_joints_(num_joints) {
  if (msg.positions.size() == num_joints_) {
    positions = Eigen::Map<Eigen::VectorXd>(
        msg.positions.data(), static_cast<Eigen::Index>(msg.positions.size()));
  } else {
    positions = Eigen::VectorXd::Zero(num_joints_);
  }

  if (msg.velocities.size() == num_joints_) {
    velocities = Eigen::Map<Eigen::VectorXd>(
        msg.velocities.data(),
        static_cast<Eigen::Index>(msg.velocities.size()));
  } else {
    velocities = Eigen::VectorXd::Zero(num_joints_);
  }
}

}  // namespace aic_controller
