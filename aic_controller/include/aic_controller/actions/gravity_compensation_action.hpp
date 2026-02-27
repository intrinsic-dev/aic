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

#ifndef AIC_CONTROLLER__ACTIONS__GRAVITY_COMPENSATION_ACTION_HPP_
#define AIC_CONTROLLER__ACTIONS__GRAVITY_COMPENSATION_ACTION_HPP_

#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/solveri.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "rclcpp/node_interfaces/node_logging_interface.hpp"

// The versions conditioning is added here to support the
// source-compatibility with ROS 2 Jazzy
#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(29, 0, 0)
#include "urdf/model.hpp"
#elif
#include "urdf/model.h"
#endif

//==============================================================================
namespace aic_controller {

//==============================================================================
/**
 * The GravityCompensationAction computes the torques required to compensate for
 * gravitional effects on the robot arm links.
 * This code has been adapted from the franka_ros2 respository which is
 * attributed to Franka Robotics GmbH. Link:
 * https://github.com/frankarobotics/franka_ros2/blob/humble/franka_gazebo/franka_ign_ros2_control/include/ign_ros2_control/model_kdl.h
 */
class GravityCompensationAction {
 public:
  GravityCompensationAction(std::size_t num_joints);

  /**
   * @brief Configure the gravity compensation action using the urdf_model, base
   * frame and tip frame to generate the kinematic chain object.
   *
   * @param urdf_model urdf model of the robot arm
   * @param base frame of the robot base link
   * @param tip frame of the end-effector tool tip
   * @return true Configure successful
   * @return false Configure failed
   */
  [[nodiscard]]
  bool configure(const urdf::Model& urdf_model, const std::string& base,
                 const std::string& tip,
                 const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr&
                     logging_if);

  /**
   * @brief Compute the torques to compensate for gravitational effects on the
   * robot arm links.
   *
   * @param current_joint_positions Current position of robot joints
   * @param output_joint_torques Computed output torques
   * @return true Computation successful
   * @return false Computation failed
   */
  bool compute(const Eigen::VectorXd& current_joint_positions,
               Eigen::VectorXd& output_joint_torques);

 private:
  const KDL::Vector gravity_vector_;

  const std::size_t num_joints_;
  KDL::Chain chain_;  // Robot's kinematic chain

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_if_;
};

}  // namespace aic_controller

#endif  // AIC_CONTROLLER__ACTIONS__GRAVITY_COMPENSATION_ACTION_HPP_
