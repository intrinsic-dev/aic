
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

  this->rosNode = rclcpp::Node::make_shared("reset_joints_node");
  const rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  // Subscribe to joint_states to get initial positions
  this->jointStateSub =
      this->rosNode->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", reliable_qos,
          [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            if (this->initialJointPositions.empty()) {
              for (size_t i = 0; i < msg->name.size(); ++i) {
                this->initialJointPositions[msg->name[i]] = msg->position[i];
                gzmsg << "Stored initial position for joint " << msg->name[i]
                      << ": " << msg->position[i] << std::endl;
              }
            }
          });
  // Subscribe to reset joint requests
  this->resetJointsReqSub =
      this->rosNode->create_subscription<aic_control_interfaces::msg::ResetJoints>(
          "/reset_joints", reliable_qos,
          [this](const aic_control_interfaces::msg::ResetJoints::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(this->mutex);
            for (const auto& joint_name : msg->joint_names) {
              gzmsg << "Received reset request for joint: " << joint_name
                    << std::endl;
              this->requestedJoints.push_back(joint_name);
            }
            this->requestId = msg->request_id;
          });
  // Publish results for reset joint requests
  this->resetJointsResPub =
      this->rosNode->create_publisher<std_msgs::msg::String>(
          "/reset_joints_result", reliable_qos);

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

  if (this->requestedJoints.empty()) {
    return;
  }
  if (!this->requestId.has_value()) {
    return;
  }

  std::lock_guard<std::mutex> lock(this->mutex);
  for (const auto& jointName : this->requestedJoints) {
    auto jointEntity = this->model.JointByName(_ecm, jointName);

    // Get initial position for this joint, default to 0.0 if not found
    double initialPosition = 0.0;
    auto it = this->initialJointPositions.find(jointName);
    if (it != this->initialJointPositions.end()) {
      initialPosition = it->second;
    } else {
      gzmsg << "Warning: No initial position found for joint " << jointName
            << ", using 0.0" << std::endl;
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
  this->requestedJoints.clear();
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
