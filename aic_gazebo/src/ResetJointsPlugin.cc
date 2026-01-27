
/*
 * Copyright (C) 2026 Intrinsic Innovation LLC
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

#include "ResetJointsPlugin.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensor.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>

GZ_ADD_PLUGIN(aic_gazebo::ResetJointsPlugin, gz::sim::System,
              aic_gazebo::ResetJointsPlugin::ISystemConfigure,
              aic_gazebo::ResetJointsPlugin::ISystemPreUpdate,
              aic_gazebo::ResetJointsPlugin::ISystemReset)

namespace aic_gazebo {
//////////////////////////////////////////////////
void ResetJointsPlugin::Configure(
    const gz::sim::Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& /*_ecm*/,
    gz::sim::EventManager& /*_eventManager*/) {
  gzdbg << "aic_gazebo::ResetJointsPlugin::Configure on entity: " << _entity
        << std::endl;

  // Initialize system update period.
  double rate = _sdf->Get<double>("update_rate", 1).first;
  std::chrono::duration<double> period{rate > 0 ? 1 / rate : 0};
  this->updatePeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

  this->model = gz::sim::Model(_entity);
  // this->topic = _sdf->Get<std::string>("topic", "/aic/reset_joints").first;
  this->topic = "/reset_joints";
  // 1. Initialize ROS 2
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  this->ros_node_ = rclcpp::Node::make_shared("reset_joints_node");
  // this->ros_node_ = ros_gz_sim::Node::Get(_sdf);
  this->reset_sub_ =
      this->ros_node_
          ->create_subscription<aic_control_interfaces::msg::ResetJoints>(
              this->topic, 10, [this](const aic_control_interfaces::msg::ResetJoints& msg) {
                // std::lock_guard<std::mutex> lock(this->mutex_);
                gzmsg << "---------- hello" << std::endl;
                for (const auto& joint_name : msg.joint_names) {
                  gzmsg << "Received reset request for joint: " << joint_name << std::endl;
                  this->requestedJoints.push_back(joint_name);
                }
              });
  // std::bind(&ResetJointsPlugin::OnResetReq, this,
  //           std::placeholders::_1));

  gzmsg << "Initialized ResetJointsPlugin!" << std::endl;
}

//////////////////////////////////////////////////
void ResetJointsPlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                                  gz::sim::EntityComponentManager& _ecm) {
  // Throttle update rate.
  auto elapsed = _info.simTime - this->lastUpdateTime;
  if (elapsed > std::chrono::steady_clock::duration::zero() &&
      elapsed < this->updatePeriod) {
    return;
  }
  this->lastUpdateTime = _info.simTime;

  /////
  std::lock_guard<std::mutex> lock(this->mutex_);
  for (const auto& jointName : this->requestedJoints) {
    auto jointEntity = this->model.JointByName(_ecm, jointName);
    // Apply the reset components
    // We set position to 0.0 and velocity to 0.0 as an example
    _ecm.SetComponentData<gz::sim::components::JointPositionReset>(jointEntity,
                                                                   {0.0});
    _ecm.SetComponentData<gz::sim::components::JointVelocityReset>(jointEntity,
                                                                   {0.0});
    gzmsg << "Joint " << jointName << " reset triggered!" << std::endl;
  }
  this->requestedJoints.clear();
}

//////////////////////////////////////////////////
void ResetJointsPlugin::Reset(const gz::sim::UpdateInfo& /*_info*/,
                              gz::sim::EntityComponentManager& /*_ecm*/) {
  gzdbg << "aic_gazebo::ResetJointsPlugin::Reset" << std::endl;
}

//////////////////////////////////////////////////
void OnResetReq(const aic_control_interfaces::msg::ResetJoints& _msg) {
  // std::lock_guard<std::mutex> lock(this->mutex_);

  // for (const auto& joint_name : _msg.joint_names) {
  //   gzmsg << "Received reset request for joint: " << joint_name << std::endl;
  //   this->requestedJoints.push_back(joint_name);
  // }
}
}  // namespace aic_gazebo
