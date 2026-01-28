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
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
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
  gz::sim::Model model;

  std::shared_ptr<rclcpp::Node> rosNode;

  /// \brief A subscriber to receive joint reset commands.
 private:
  rclcpp::Subscription<aic_control_interfaces::msg::ResetJoints>::SharedPtr
      resetJointsReqSub;

  /// \brief A subscriber to receive joint state messages for initial positions.
 private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub;

  /// \brief A publisher to publish joint state reset result.
 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr resetJointsResPub;

  /// \brief Current reset request ID.
 private:
  std::optional<std::string> requestId;

  /// \brief List of joints requested to be reset.
 private:
  std::vector<std::string> requestedJoints;

  /// \brief Map of joint names to their initial positions.
 private:
  std::unordered_map<std::string, double> initialJointPositions;

  /// \brief Mutex to prevent overwriting joint requests.
 private:
  std::mutex mutex;

  /// \brief Thread to spin ROS 2 node.
 private:
  std::thread spinThread;

  /// \brief System update period calculated from <update_rate>.
 private:
  std::chrono::steady_clock::duration updatePeriod{0};

  /// \brief Last system update simulation time.
 private:
  std::chrono::steady_clock::duration lastUpdateTime{0};
};
}  // namespace aic_gazebo
#endif
