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

#include "aic_control_interfaces/msg/joint_motion_update.hpp"
#include "aic_control_interfaces/msg/motion_update.hpp"
#include "aic_control_interfaces/msg/trajectory_generation_mode.hpp"
#include "aic_engine_interfaces/srv/start_engine.hpp"
#include "aic_engine_interfaces/srv/stop_engine.hpp"
#include "aic_scoring/ScoringTier2.hh"
#include "aic_scoring/TierScore.hh"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "yaml-cpp/yaml.h"

//==============================================================================
namespace aic {

using Connection = aic_scoring::Connection;
using CableConnections = aic_scoring::CableConnections;
using JointMotionUpdateMsg = aic_control_interfaces::msg::JointMotionUpdate;
using MotionUpdateMsg = aic_control_interfaces::msg::MotionUpdate;
using StartEngineSrv = aic_engine_interfaces::srv::StartEngine;
using StopEngineSrv = aic_engine_interfaces::srv::StopEngine;
using TriggerSrv = std_srvs::srv::Trigger;

//==============================================================================
struct TrialScore {
  aic_scoring::Tier1Score tier_1;
  aic_scoring::Tier2Score tier_2;
  aic_scoring::Tier3Score tier_3;

  TrialScore()
      : tier_1(0),
        tier_2("Task execution failed"),
        tier_3(0, "Task execution failed") {}

  void tier_1_success() { tier_1 = aic_scoring::Tier1Score(1); }

  double total_score() const {
    return tier_1.total_score() + tier_2.total_score() + tier_3.total_score();
  }
};

//==============================================================================
enum class TaskStatus {
  // The task was started, stopping it will initialize scoring.
  STARTED = 0,
  // The task finished, it will be scored and printed
  FINISHED = 1
};

//==============================================================================
struct TrialAttempt {
  CableConnections task;
  TaskStatus status;
  TrialScore score;
  TrialAttempt(const CableConnections& task_)
      : task(task_), status(TaskStatus::STARTED) {}
};

//==============================================================================
// Ensure rclcpp::init has been called before creating an instance.
class Engine {
 public:
  /// \brief Constructor.
  Engine(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// \brief Spin the inner node.
  void spin();

 private:
  // Initializes the engine.
  /// \return An error message if initialization failed, std::nullopt otherwise.
  std::optional<std::string> initialize();

  /// \brief Handle the logic for a given trial.
  /// \param[in] task The task to score.
  /// \return An error message if an error occured, std::nullopt otherwise.
  std::optional<std::string> start_trial(const CableConnections& task, const uint64_t time_limit);

  /// \brief Fully resets the engine to prepare for a new set of tasks.
  void reset_engine();

  /// \brief Check if the participant model is ready. As per challenge
  /// requirements. See challenge_rules.md for details. \return True if the
  /// model is ready, false otherwise.
  bool check_model();

  /// \brief Check if required endpoints are available.
  /// \return True if all required endpoints are available, false otherwise.
  bool check_endpoints();

  /// \brief Check if the scoring system is ready.
  /// \return True if the scoring system is ready, false otherwise.
  bool ready_scoring(const CableConnections& task, const uint64_t time_limit);

  /// @brief Stop the bag recording and compute the current score.
  /// @param[in] A reference to the current trial score to update.
  void compute_score(TrialScore& trial);

  /// @brief Writes the result of the current run to a YAML file.
  /// \param[in] The score to serialize and write.
  void score_run();

  /// @brief Callback for the service to start the engine. Will start scoring.
  void start_engine_callback(
      const std::shared_ptr<StartEngineSrv::Request> request,
      std::shared_ptr<StartEngineSrv::Response> response);

  /// @brief Callback for the service to stop the engine. Will stop the scoring
  /// and optionally print and return the result of all trials.
  void stop_engine_callback(
      const std::shared_ptr<StopEngineSrv::Request> request,
      std::shared_ptr<StopEngineSrv::Response> response);

  // Strings.
  // Name of the aic_adapter node for lifecycle transitions.
  std::string adapter_node_name_;
  // Name of the participant's model node for lifecycle transitions.
  std::string model_node_name_;
  // Name of the service to get the lifecycle state of the model node.
  std::string model_get_state_service_name_;

  // Internal ROS 2 node.
  rclcpp::Node::SharedPtr node_;

  // Service clients.
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr
      model_get_state_client_;
  rclcpp::Client<TriggerSrv>::SharedPtr tare_ft_client_;

  // Service servers.
  rclcpp::Service<StartEngineSrv>::SharedPtr start_engine_server_;
  rclcpp::Service<StopEngineSrv>::SharedPtr stop_engine_server_;

  // Callback group for concurrent execution.
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Set to true when a run is requested, will be processed by main thread.
  // Set to false when a run is finished and the engine is idle.
  bool requested_run_ = false;

  // Task configs. Key is id, value is the task.
  std::map<std::string, TrialAttempt> tasks_;

  // Parameters to skip states for testing purposes.
  bool skip_model_ready_;
  bool skip_ready_simulator_;

  // Scoring tier 2 instance.
  std::unique_ptr<aic_scoring::ScoringTier2> scoring_tier2_;

  // Output directory for scoring.
  std::string scoring_output_dir_;
};

}  // namespace aic

#endif  // AIC_ENGINE_HPP_
