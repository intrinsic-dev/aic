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

#ifndef AIC_ENGINE_HPP_
#define AIC_ENGINE_HPP_

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "aic_task_interfaces/action/insert_cable.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "simulation_interfaces/srv/spawn_entity.hpp"
#include "yaml-cpp/yaml.h"

//==============================================================================
namespace aic {

using InsertCableAction = aic_task_interfaces::action::InsertCable;
using InsertCableGoalHandle =
    rclcpp_action::ServerGoalHandle<InsertCableAction>;
using JointStateMsg = sensor_msgs::msg::JointState;
using SpawnEntitySrv = simulation_interfaces::srv::SpawnEntity;
using Task = aic_task_interfaces::msg::Task;
using WrenchStampedMsg = geometry_msgs::msg::WrenchStamped;

//==============================================================================
enum class EngineState : uint8_t {
  Uninitialized = 0,
};

//==============================================================================
enum class TrialState : uint8_t {
  Uninitialized = 0,
  Configured,
  SimulatorReady,
  TaskStarted,
  TaskCompleted
};

//==============================================================================
struct Trial {
  // Constructor.
  Trial(const std::string& id, const std::string& cable_type,
        const std::string& cable_name, const std::string& plug_type,
        const std::string& plug_name, const std::string& port_type,
        const std::string& port_name, const std::string& target_module_name,
        std::size_t time_limit);

  Task task;
  TrialState state;
};

//==============================================================================
class Engine : public rclcpp::Node {
 public:
  /// \brief Constructor.
  Engine(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// \brief Spawn the task board in Gazebo.
  /// \param[in] x X position
  /// \param[in] y Y position
  /// \param[in] z Z position
  /// \param[in] roll Roll orientation (radians)
  /// \param[in] pitch Pitch orientation (radians)
  /// \param[in] yaw Yaw orientation (radians)
  /// \return True if spawning succeeded, false otherwise
  bool spawn_task_board(double x, double y, double z, double roll, double pitch,
                        double yaw);

  /// \brief Destructor.
  ~Engine();

 private:
  // Subscriptions.
  rclcpp::Subscription<WrenchStampedMsg>::SharedPtr wrench_sub_;
  rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;

  // Publishers.

  // Action clients.
  rclcpp_action::Client<InsertCableAction>::SharedPtr
      insert_cable_action_client_;

  // Service clients.
  rclcpp::Client<SpawnEntitySrv>::SharedPtr spawn_entity_client_;

  // Strings.
  // Name of the aic_adapter node for lifecycle transitions.
  std::string adapter_node_name_;
  // Name of the participant's model node for lifecycle transitions.
  std::string model_node_name_;

  // Task config.
  YAML::Node config_;

  // The active trial.
  std::optional<Trial> active_trial_;

  // Task thread.
  std::thread task_thread_;
};

}  // namespace aic

#endif  // AIC_ENGINE_HPP_
