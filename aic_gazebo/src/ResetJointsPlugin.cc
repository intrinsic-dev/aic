
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

#include <aic_engine_interfaces/srv/reset_joints.hpp>
#include <future>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocityReset.hh>

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
  this->updatePeriod_ =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

  this->model_ = gz::sim::Model(_entity);
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  this->rosNode_ = rclcpp::Node::make_shared("reset_joints_node");
  this->reset_joints_srv_ =
      this->rosNode_->create_service<aic_engine_interfaces::srv::ResetJoints>(
          "/scoring/reset_joints",
          [this](
              const std::shared_ptr<
                  aic_engine_interfaces::srv::ResetJoints::Request>
                  request,
              std::shared_ptr<aic_engine_interfaces::srv::ResetJoints::Response>
                  response) {
            // Validate request parameters (no mutex needed)
            if (request->joint_names.empty() ||
                request->initial_positions.empty()) {
              // Reject empty request
              response->success = false;
              response->message = "Reset joints request is empty!";
              return;
            }
            const auto num_joints = request->joint_names.size();
            if (num_joints != request->initial_positions.size()) {
              // Reject request, they should have the same length
              response->success = false;
              response->message =
                  "Number of joint names and initial positions provided should "
                  "be the same!";
              return;
            }

            // Check busy state and set up request
            std::shared_ptr<
                std::future<aic_engine_interfaces::srv::ResetJoints::Response>>
                future_ptr;
            {
              std::lock_guard<std::mutex> lock(this->mutex_);

              if (!this->requestedJoints_.empty() || this->reset_promise_) {
                // Reject request, another reset request is ongoing
                response->success = false;
                response->message = "ResetJoints service is busy!";
                return;
              }

              this->reset_promise_ = std::make_shared<std::promise<
                  aic_engine_interfaces::srv::ResetJoints::Response>>();
              future_ptr = std::make_shared<std::future<
                  aic_engine_interfaces::srv::ResetJoints::Response>>(
                  this->reset_promise_->get_future());

              for (std::size_t i = 0; i < num_joints; ++i) {
                const auto& jointName = request->joint_names[i];
                const auto& initialPosition = request->initial_positions[i];
                gzmsg << "Received reset request for joint: " << jointName
                      << std::endl;
                this->requestedJoints_[jointName] = initialPosition;
              }
            }

            // Wait for PreUpdate to complete reset (outside mutex)
            *response = future_ptr->get();
          });

  this->spinThread_ = std::thread([this]() { rclcpp::spin(this->rosNode_); });

  gzmsg << "Initialized ResetJointsPlugin!" << std::endl;
}

//////////////////////////////////////////////////
void ResetJointsPlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                                  gz::sim::EntityComponentManager& _ecm) {
  // Throttle update rate.
  auto elapsed = _info.simTime - this->lastUpdateTime_;
  if (elapsed > std::chrono::steady_clock::duration::zero() &&
      elapsed < this->updatePeriod_) {
    return;
  }
  this->lastUpdateTime_ = _info.simTime;

  std::lock_guard<std::mutex> lock(this->mutex_);
  if (this->requestedJoints_.empty()) {
    return;
  }

  for (const auto& [jointName, initialPosition] : this->requestedJoints_) {
    auto jointEntity = this->model_.JointByName(_ecm, jointName);
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

  aic_engine_interfaces::srv::ResetJoints::Response response;
  response.success = true;
  this->reset_promise_->set_value(response);

  this->requestedJoints_.clear();
  this->reset_promise_ = nullptr;
}

//////////////////////////////////////////////////
void ResetJointsPlugin::Reset(const gz::sim::UpdateInfo& /*_info*/,
                              gz::sim::EntityComponentManager& /*_ecm*/) {
  gzdbg << "aic_gazebo::ResetJointsPlugin::Reset" << std::endl;
}

//////////////////////////////////////////////////
ResetJointsPlugin::~ResetJointsPlugin() {
  if (this->spinThread_.joinable()) {
    rclcpp::shutdown();
    this->spinThread_.join();
  }
}

}  // namespace aic_gazebo
