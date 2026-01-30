
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
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  // Taken from ur_gz.urdf.xacro
  this->initialJointPositions = {
    {"shoulder_pan_joint": -0.546},
    {"shoulder_lift_joint": -1.703},
    {"elbow_joint": -1.291},
    {"wrist_1_joint": -1.719},
    {"wrist_2_joint": 1.571},
    {"wrist_3_joint": -2.116},
  };

  this->rosNode = rclcpp::Node::make_shared("reset_joints_node");
  const rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  const rclcpp::QoS transient_qos =
      rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
  // Subscribe to reset joint requests
  this->resetJointsReqSub = this->rosNode->create_subscription<
      aic_control_interfaces::msg::ResetJoints>(
      "/reset_joints", reliable_qos,
      [this](const aic_control_interfaces::msg::ResetJoints::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->requestId = msg->request_id;
      });
  // Publish home joint states
  this->homeJointStatePub =
      this->rosNode->create_publisher<sensor_msgs::msg::JointState>(
          "/home_joint_states", transient_qos);
  // Publish results for reset joint requests
  this->resetJointsResPub =
      this->rosNode->create_publisher<std_msgs::msg::String>(
          "/reset_joints_result", reliable_qos);

  // Publish transient local initial positions for late joiners
  sensor_msgs::msg::JointState initial_state;
  for (const auto& [jointName, initialPosition] : this->initialJointPositions) {
    initial_state.name.emplace_back(jointName);
    initial_state.position.emplace_back(initialPosition);
  }
  this->homeJointStatePub->publish(initial_state);

  this->spinThread = std::thread([this]() { rclcpp::spin(this->rosNode); });

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

  std::lock_guard<std::mutex> lock(this->mutex);
  // Reset joint state subscriber if initial positions have been retrieved
  if (this->jointStateSub && !this->initialJointPositions.empty()) {
    this->jointStateSub.reset();
  }

  if (!this->requestId.has_value()) {
    return;
  }

  for (const auto& [jointName, initialPosition] : this->initialJointPositions) {
    auto jointEntity = this->model.JointByName(_ecm, jointName);
    if (!jointEntity) {
      gzwarn << "Joint " << jointName << " cannot be found! Skipping reset."
             << std::endl;
      continue;
    }

    // Apply the reset components using initial position
    _ecm.SetComponentData<gz::sim::components::JointPositionReset>(
        jointEntity, {initialPosition});
    _ecm.SetComponentData<gz::sim::components::JointVelocityReset>(jointEntity,
                                                                   {0.0});
    gzmsg << "Joint " << jointName
          << " reset to initial position: " << initialPosition << std::endl;
  }
  std_msgs::msg::String result;
  result.data = this->requestId.value();
  this->resetJointsResPub->publish(result);
  this->requestId = std::nullopt;
}

//////////////////////////////////////////////////
void ResetJointsPlugin::Reset(const gz::sim::UpdateInfo& /*_info*/,
                              gz::sim::EntityComponentManager& /*_ecm*/) {
  gzdbg << "aic_gazebo::ResetJointsPlugin::Reset" << std::endl;
}

//////////////////////////////////////////////////
ResetJointsPlugin::~ResetJointsPlugin() {
  if (this->spinThread.joinable()) {
    rclcpp::shutdown();
    this->spinThread.join();
  }
}

}  // namespace aic_gazebo
