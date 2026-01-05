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
#include <unordered_map>

#include "aic_task_interfaces/action/insert_cable.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "simulation_interfaces/srv/delete_entity.hpp"
#include "simulation_interfaces/srv/spawn_entity.hpp"
#include "yaml-cpp/yaml.h"

//==============================================================================
namespace aic {

using DeleteEntitySrv = simulation_interfaces::srv::DeleteEntity;
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
  Initialized,
  Running,
  Error,
  Completed
};

//==============================================================================
// For each trial, track its state.
// States progress from Uninitialized -> EndpointsAvailable -> SimulatorReady
// ->ScoringReady -> TaskStarted -> TaskCompleted
// Uninitialized: Trial has not started.
// EndpointsAvailable: Required nodes are up and running.
// SimulatorReady: Simulator is ready with the task board and cables spawned.
// TaskStarted: Task goal has been sent to the participant model. Clock started.
// TaskCompleted: Task has been completed successfully or time limit reached.
enum class TrialState : uint8_t {
  Uninitialized = 0,
  EndpointsAvailable,
  SimulatorReady,
  ScoringReady,
  TaskStarted,
  TaskCompleted
};

//==============================================================================
struct Trial {
  // Constructor.
  // Throws std::runtime_error error if config is invalid.
  Trial(const std::string& id, YAML::Node config);

  std::string id;
  std::optional<std::string> spawned_task_board_name;
  YAML::Node config;
  std::unordered_map<std::string, Task> tasks;  // Map of task_id -> Task
  TrialState state;
};

//==============================================================================
// Ensure rclcpp::init has been called before creating an instance.
class Engine {
 public:
  /// \brief Constructor.
  Engine(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// \brief Destructor.
  ~Engine();

  /// \brief Start the engine.
  void start();

 private:
  // Initializes the engine.
  EngineState initialize();

  /// \brief Run the engine.
  EngineState run();

  /// \brief Handle the logic for a given trial.
  /// \param[in] trial The trial to handle.
  /// \return The resulting state of the trial after handling.
  TrialState handle_trial(const Trial& trial);

  /// \brief Reset internal and simulator states after a trial is completed.
  void reset_after_trial();

  /// \brief Check if required endpoints are available.
  /// \return True if all required endpoints are available, false otherwise.
  bool check_required_endpoints();

  /// \brief Check if the simulator is ready.
  /// \return True if the simulator is ready, false otherwise.
  bool ready_simulator();

  /// \brief Check if the scoring system is ready.
  /// \return True if the scoring system is ready, false otherwise.
  bool ready_scoring();

  /// \brief Start the task.
  /// \return True if the task started successfully, false otherwise.
  bool start_task();

  /// \brief Check if the task was completed successfully.
  /// \return True if the task was completed successfully, false otherwise.
  bool task_completed_successfully();

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

  // Internal ROS 2 node.
  rclcpp::Node::SharedPtr node_;
  // Subscriptions.
  rclcpp::Subscription<WrenchStampedMsg>::SharedPtr wrench_sub_;
  rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;

  // Publishers.

  // Action clients.
  rclcpp_action::Client<InsertCableAction>::SharedPtr
      insert_cable_action_client_;

  // Service clients.
  rclcpp::Client<SpawnEntitySrv>::SharedPtr spawn_entity_client_;
  rclcpp::Client<DeleteEntitySrv>::SharedPtr delete_entity_client_;

  // Strings.
  // Name of the aic_adapter node for lifecycle transitions.
  std::string adapter_node_name_;
  // Name of the participant's model node for lifecycle transitions.
  std::string model_node_name_;

  // Task config.
  YAML::Node config_;

  // All trials parsed from config.
  std::vector<std::pair<std::string, Trial>> trials_;

  // The active trial.
  std::optional<Trial> active_trial_;

  // Thread to spin ROS 2 node.
  std::thread spin_thread_;

  // Engine state.
  EngineState engine_state_;
};

}  // namespace aic

#endif  // AIC_ENGINE_HPP_
