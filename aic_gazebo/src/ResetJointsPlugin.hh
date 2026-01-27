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

#ifndef AIC_GAZEBO__RESET_JOINTS_PLUGIN_HH_
#define AIC_GAZEBO__RESET_JOINTS_PLUGIN_HH_

#include <aic_control_interfaces/msg/reset_joints.hpp>
#include <chrono>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

namespace aic_gazebo {
class ResetJointsPlugin : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate,
                          public gz::sim::ISystemReset {
  // Documentation inherited
 public:
  void Configure(const gz::sim::Entity& _entity,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 gz::sim::EntityComponentManager& _ecm,
                 gz::sim::EventManager& _eventManager) override;

  // Documentation inherited
 public:
  void PreUpdate(const gz::sim::UpdateInfo& _info,
                 gz::sim::EntityComponentManager& _ecm) override;

  // Documentation inherited
 public:
  void Reset(const gz::sim::UpdateInfo& _info,
             gz::sim::EntityComponentManager& _ecm) override;

 private:
  void OnResetReq(const aic_control_interfaces::msg::ResetJoints& _msg);

  /// \brief The model associated with this system.
 private:
  gz::sim::Model model;

  std::shared_ptr<rclcpp::Node> ros_node_;

  /// \brief A subscriber to receive joint reset commands.
 private:
  rclcpp::Subscription<aic_control_interfaces::msg::ResetJoints>::SharedPtr
      reset_sub_;

  /// \brief The topic to receive joint reset commands.
 private:
  std::string topic;

  /// \brief List of joints requested to be reset.
 private:
  std::vector<std::string> requestedJoints;

  std::mutex mutex_;

  /// \brief System update period calculated from <update_rate>.
 private:
  std::chrono::steady_clock::duration updatePeriod{0};

  /// \brief Last system update simulation time.
 private:
  std::chrono::steady_clock::duration lastUpdateTime{0};
};
}  // namespace aic_gazebo
#endif
