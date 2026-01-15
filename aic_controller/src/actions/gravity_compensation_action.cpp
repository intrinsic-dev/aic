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

#include "aic_controller/actions/gravity_compensation_action.hpp"

namespace aic_controller {

//==============================================================================
GravityCompensationAction::GravityCompensationAction(std::size_t num_joints)
    : gravity_vector_(KDL::Vector(0.0, 0.0, -9.80665)),
      num_joints_(num_joints) {}

//==============================================================================
bool GravityCompensationAction::configure(
    const urdf::Model& urdf_model, const std::string& base,
    const std::string& tip,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_if

) {
  logging_if_ = logging_if;

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
    RCLCPP_ERROR(logging_if_->get_logger(),
                 "Cannot construct KDL tree from URDF");
    return false;
  }
  if (!tree.getChain(base, tip, chain_)) {
    RCLCPP_ERROR(
        logging_if_->get_logger(),
        "Cannot find chain within URDF tree from root '%s' to tip '%s'.",
        base.c_str(), tip.c_str());
    return false;
  }

  return true;
}

//==============================================================================
bool GravityCompensationAction::compute(
    const Eigen::VectorXd& current_joint_positions,
    Eigen::VectorXd& output_joint_torques) {
  KDL::JntArray joint_positions(num_joints_);
  joint_positions.data = current_joint_positions;

  KDL::Chain chain =
      this->chain_;  // Make a copy of the chain at this point in time
  KDL::ChainDynParam solver(chain, gravity_vector_);

  KDL::JntArray gravity_compensation_torques(num_joints_);
  int error =
      solver.JntToGravity(joint_positions, gravity_compensation_torques);
  if (error != KDL::SolverI::E_NOERROR) {
    RCLCPP_ERROR(logging_if_->get_logger(),
                 "Error encountered by KDL::ChainDynParam when solving for "
                 "gravitational forces on the joints");
    return false;
  }

  output_joint_torques = gravity_compensation_torques.data;

  return true;
}

}  // namespace aic_controller
