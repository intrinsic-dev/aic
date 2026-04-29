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

#include <aic_engine_interfaces/srv/start_engine.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

#include "aic_control_interfaces/msg/joint_motion_update.hpp"
#include "aic_control_interfaces/msg/motion_update.hpp"
#include "aic_control_interfaces/msg/trajectory_generation_mode.hpp"
#include "aic_scoring/ScoringTier2.hh"
#include "aic_scoring/TierScore.hh"
#include "aic_task_interfaces/action/insert_cable.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "yaml-cpp/yaml.h"

#include <grpcpp/grpcpp.h>
#include "intrinsic/executive/proto/executive_service.grpc.pb.h"

//==============================================================================
namespace aic {

using ExecutiveService = intrinsic_proto::executive::ExecutiveService;
using InsertCableAction = aic_task_interfaces::action::InsertCable;
using JointMotionUpdateMsg = aic_control_interfaces::msg::JointMotionUpdate;
using MotionUpdateMsg = aic_control_interfaces::msg::MotionUpdate;
using StartEngineSrv = aic_engine_interfaces::srv::StartEngine;
using TriggerSrv = std_srvs::srv::Trigger;

//==============================================================================
struct Score {
  // Tier_1 done only once for the whole run
  aic_scoring::Tier1Score tier_1;

  void tier_1_success() { tier_1 = aic_scoring::Tier1Score(1); }

  // Intentionally alphabetically sorted, trial_id -> category scores
  std::map<std::string, aic_scoring::Tier2Score> breakdown;

  /// \brief Serializes the score into a YAML node for logging.
  /// \return The resulting YAML node with serialized data.
  YAML::Node serialize() const;

  /// \brief Computes the total score from the score breakdown.
  double calculate_total_score() const;
};

//==============================================================================
struct TrialResult {
  // Score for the run
  Score score;
  // Error message if there was an error, std::nullopt otherwise.
  std::optional<std::string> error;

  TrialResult(const Score& score_, const std::optional<std::string>& error_ = std::nullopt) :
    score(score_), error(error_) { }
};

//==============================================================================
// Ensure rclcpp::init has been called before creating an instance.
class Engine {
 public:
  /// \brief Constructor.
  Engine(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// \brief Processes incoming requests. Blocking call will only exit when
  /// rclcpp::ok() returns false;
  void process();

  /// \brief Destructor.
  ~Engine();

 private:
  // Initializes the engine.
  /// \param[in] trial The yaml configuration for the run.
  /// \return An error message if initialization failed, std::nullopt otherwise.
  std::optional<std::string> initialize(const std::string& yaml_config);

  /// \brief Run the engine.
  /// \return An error message if execution failed, std::nullopt otherwise.
  std::optional<std::string> run();

  /// \brief Handle the logic for a given trial.
  /// \param[in] trial The trial to handle.
  /// \return The result of the trial after handling.
  TrialResult handle_trial();

  /// \brief Reset internal and simulator states after a trial is completed.
  /// \param[in] trial The trial currently being ran
  void reset_after_trial();

  /// \brief Check if the participant model is ready. As per challenge
  /// requirements. See challenge_rules.md for details. \return True if the
  /// model is ready, false otherwise.
  bool check_model();

  /// \brief Check if required endpoints are available.
  /// \return True if all required endpoints are available, false otherwise.
  bool check_endpoints();

  /// \brief Check if the scoring system is ready.
  /// \return True if the scoring system is ready, false otherwise.
  bool ready_scoring();

  /// \brief Check if tasks were started successfully.
  /// \return True if tasks were started successfully, false otherwise.
  bool tasks_started();

  /// \brief Check if all tasks have been completed successfully.
  /// \return True if tasks were completed successfully, false otherwise.
  bool tasks_completed_successfully();

  /// @brief Check if the robot was commanded to move by the model node.
  /// @return True if the robot was commanded to move, false otherwise.
  bool model_node_moved_robot();

  /// @brief Check if the model is in the unconfigured state together with other
  /// expectations in this state.
  /// @return True if the model is unconfigured, false otherwise.
  bool model_node_is_unconfigured();

  /// @brief Trigger a state transition for the lifecycle node.
  /// \param[in] transition The transition to trigger as per
  /// lifecycle_msgs::msg::Transition enum definition.
  /// @return True if transition succeeded, false otherwise.
  bool transition_model_lifecycle_node(const uint8_t transition);

  /// @brief Configure the model node and check expectations in the configured
  /// state as per challenge requirements.
  /// @return True if configuration succeeded, false otherwise.
  bool configure_model_node();

  /// @brief Activate the model node to transition from configured to active
  /// state.
  /// @return True if activation succeeded, false otherwise.
  bool activate_model_node();

  /// @brief Deactivate the model node to transition from active to configured
  /// state.
  /// @return True if deactivation succeeded, false otherwise.
  bool deactivate_model_node();

  /// @brief Cleanup the model node to transition from inactive to
  /// unconfigured state.
  /// @return True if cleanup succeeded, false otherwise.
  bool cleanup_model_node();

  /// @brief Stop the bag recording and compute the current score.
  /// @param[in] A reference to the current trial score to update.
  void compute_score(Score& trial);

  /// @brief Writes the result of the current run to a YAML file.
  /// \param[in] The score to serialize and write.
  void score_run(const Score& score);

  /// @brief Wait for a future, interrupt if rclcpp Context is shut down.
  /// \param[in] The future to wait for.
  /// \param[in] The timeout to wait until.
  /// @return true if the future resolved, false if it didn't.
  template <typename FutureT>
  bool wait_for_interruptible(const FutureT& future,
                              const std::chrono::seconds timeout) const {
    const auto start = node_->now();
    const auto timeout_duration = rclcpp::Duration(timeout);
    while (rclcpp::ok() && (node_->now() - start) < timeout_duration) {
      if (future.wait_for(std::chrono::milliseconds(50)) ==
          std::future_status::ready) {
        return true;
      }
    }
    return false;
  }

  /// @brief Callback for the service to start the engine. Will handle the
  /// requested trial and return a response with its outcome.
  void start_engine_callback(const std::shared_ptr<StartEngineSrv::Request> request,
      std::shared_ptr<StartEngineSrv::Response> response);

  // Strings.
  // Name of the aic_adapter node for lifecycle transitions.
  std::string adapter_node_name_;
  // Name of the participant's model node for lifecycle transitions.
  std::string model_node_name_;
  // Name of the service to get the lifecycle state of the model node.
  std::string model_get_state_service_name_;
  // Name of the service to change the lifecycle state of the model node.
  std::string model_change_state_service_name_;

  // Executive service address.
  std::string executive_service_address_;
  // Executive service stub.
  std::unique_ptr<ExecutiveService::Stub> executive_stub_;
  // Current long running operation name.
  std::string current_operation_name_;

  // Internal ROS 2 node.
  rclcpp::Node::SharedPtr node_;
  // Subscriptions.
  rclcpp::Subscription<JointMotionUpdateMsg>::SharedPtr
      joint_motion_update_sub_;
  rclcpp::Subscription<MotionUpdateMsg>::SharedPtr motion_update_sub_;

  // Subscription messages.
  JointMotionUpdateMsg::ConstSharedPtr last_joint_motion_update_msg_;
  MotionUpdateMsg::ConstSharedPtr last_motion_update_msg_;

  // Action clients.
  rclcpp_action::Client<InsertCableAction>::SharedPtr
      insert_cable_action_client_;

  // Service clients.
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr
      model_get_state_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr
      model_change_state_client_;
  rclcpp::Client<TriggerSrv>::SharedPtr tare_ft_client_;


  // Service servers.
  rclcpp::Service<StartEngineSrv>::SharedPtr start_engine_server_;

  // Callback group for concurrent execution.
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Set to true when a run is requested, will be processed by main thread.
  // Set to false when a run is finished and the engine is idle.
  std::atomic<bool> requested_run_ = false;

  // Task config.
  YAML::Node config_;

  // Thread to spin ROS 2 node.
  std::thread spin_thread_;

  // Whether to publish ground truth data for scoring.
  bool ground_truth_;

  // Parameters to skip states for testing purposes.
  bool skip_model_ready_;
  bool skip_ready_simulator_;

  // Whether the participant model has been discovered and readied.
  bool model_discovered_;

  // Scoring tier 2 instance.
  std::unique_ptr<aic_scoring::ScoringTier2> scoring_tier2_;

  // Output directory for scoring.
  std::string scoring_output_dir_;
};

}  // namespace aic

#endif  // AIC_ENGINE_HPP_
