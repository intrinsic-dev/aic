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

#include <aic_engine_interfaces/srv/reset_joints.hpp>
#include <chrono>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

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

 public:
  ~ResetJointsPlugin();

  /// \brief The model associated with this system.
 private:
  gz::sim::Model model_;

  /// \brief ROS 2 node to interact with the engine.
 private:
  std::shared_ptr<rclcpp::Node> rosNode_;

  /// \brief ROS2 Service servers for updating controller states
 private:
  rclcpp::Service<aic_engine_interfaces::srv::ResetJoints>::SharedPtr
      reset_joints_srv_;

  /// \brief Stored promise for handling reset service response between threads
 private:
  std::shared_ptr<
      std::promise<aic_engine_interfaces::srv::ResetJoints::Response>>
      reset_promise_;

  /// \brief Map of joint names to be reset to their initial positions.
 private:
  std::unordered_map<std::string, double> requestedJoints_;

  /// \brief Mutex to prevent overwriting joint requests.
 private:
  std::mutex mutex_;

  /// \brief Thread to spin ROS 2 node.
 private:
  std::thread spinThread_;

  /// \brief System update period calculated from <update_rate>.
 private:
  std::chrono::steady_clock::duration updatePeriod_{0};

  /// \brief Last system update simulation time.
 private:
  std::chrono::steady_clock::duration lastUpdateTime_{0};
};
}  // namespace aic_gazebo
#endif
