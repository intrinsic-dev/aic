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

#include "aic_controller/cartesian_state.hpp"

namespace aic_controller {

//==============================================================================
CartesianState::CartesianState()
: pose(Eigen::Isometry3d::Identity()),
	velocity(Eigen::Matrix<double, 6, 1>::Zero()) {};

//==============================================================================
CartesianState::CartesianState(
	const geometry_msgs::msg::Pose& pose_msg,
	const geometry_msgs::msg::Twist& velocity_msg) {
	tf2::fromMsg(pose_msg, pose);
	tf2::fromMsg(velocity_msg, velocity);
}

}  // namespace aic_controller
