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

#include "aic_engine.hpp"

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_set>

#include "aic_task_interfaces/msg/task.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/subscription_options.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace aic {

//==============================================================================
Engine::Engine(const rclcpp::NodeOptions& options)
    : node_(std::make_shared<rclcpp::Node>("aic_engine", options)),
      insert_cable_action_client_(nullptr) {
  RCLCPP_INFO(node_->get_logger(), "Creating AIC Engine...");

  // Declare ROS parameters.
  adapter_node_name_ = node_->declare_parameter(
      "adapter_node_name", std::string("aic_adapter_node"));
  model_node_name_ =
      node_->declare_parameter("model_node_name", std::string("aic_model"));
  model_get_state_service_name_ = "/" + model_node_name_ + "/get_state";
  model_change_state_service_name_ = "/" + model_node_name_ + "/change_state";
  node_->declare_parameter("config_file_path", std::string(""));
  node_->declare_parameter("endpoint_ready_timeout_seconds", 10);
  node_->declare_parameter("gripper_frame_name", std::string("gripper/tcp"));
  skip_model_ready_ = node_->declare_parameter("skip_model_ready", false);
  node_->declare_parameter("model_discovery_timeout_seconds", 30);
  node_->declare_parameter("model_configure_timeout_seconds", 60);
  node_->declare_parameter("model_activate_timeout_seconds", 60);
  node_->declare_parameter("model_deactivate_timeout_seconds", 60);
  node_->declare_parameter("model_cleanup_timeout_seconds", 60);
  node_->declare_parameter("model_shutdown_timeout_seconds", 60);

  // Set scoring output directory from AIC_RESULTS_DIR environment variable
  // If not set or empty, default to $HOME/aic_results
  const char* aic_results_dir = std::getenv("AIC_RESULTS_DIR");
  if (aic_results_dir != nullptr && aic_results_dir[0] != '\0') {
    scoring_output_dir_ = std::string(aic_results_dir);
  } else {
    const char* home_dir = std::getenv("HOME");
    if (home_dir != nullptr) {
      scoring_output_dir_ = std::string(home_dir) + "/aic_results";
    } else {
      RCLCPP_ERROR(node_->get_logger(),
                   "HOME environment variable not set. Cannot determine "
                   "scoring output directory.");
      throw std::runtime_error("HOME environment variable not set");
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Scoring output directory set to: %s",
              scoring_output_dir_.c_str());

  callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  start_engine_server_ = node_->create_service<StartEngineSrv>(
      "/start_aic_engine",
      std::bind(&Engine::start_engine_callback, this, std::placeholders::_1,
                std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);

  stop_engine_server_ = node_->create_service<StopEngineSrv>(
      "/stop_aic_engine",
      std::bind(&Engine::stop_engine_callback, this, std::placeholders::_1,
                std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);

  scoring_tier2_ = std::make_unique<aic_scoring::ScoringTier2>(node_.get());

  scoring_tier2_->SetGripperFrame(
      node_->get_parameter("gripper_frame_name").as_string());

  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = callback_group_;

  insert_cable_action_client_ = rclcpp_action::create_client<InsertCableAction>(
      node_, "/insert_cable", callback_group_);
  model_get_state_client_ = node_->create_client<lifecycle_msgs::srv::GetState>(
      model_get_state_service_name_, rclcpp::ServicesQoS(), callback_group_);
  model_change_state_client_ =
      node_->create_client<lifecycle_msgs::srv::ChangeState>(
          model_change_state_service_name_, rclcpp::ServicesQoS(),
          callback_group_);
  tare_ft_client_ = node_->create_client<TriggerSrv>(
      "/aic_controller/tare_force_torque_sensor", rclcpp::ServicesQoS(),
      callback_group_);
}

//==============================================================================
void Engine::spin() {
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(this->node_);
  executor.spin();
}

//==============================================================================
void Engine::start_engine_callback(
    const std::shared_ptr<StartEngineSrv::Request> request,
    std::shared_ptr<StartEngineSrv::Response> response) {
  if (request->reset) {
    this->reset_engine();
  }

  if (requested_run_) {
    const std::string error =
        "Failed starting engine, a run is already underway";
    RCLCPP_ERROR(node_->get_logger(), "%s", error.c_str());
    response->success = false;
    response->message = error;
    return;
  }

  // Only initialize once per run on the initial trial
  if (request->reset) {
    const auto initialization_result = this->initialize();
    if (initialization_result.has_value()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed initializing engine: %s",
                   initialization_result.value().c_str());
      response->success = false;
      response->message = initialization_result.value();
      return;
    }
  }

  const auto err = this->start_trial(request->task);
  if (err.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed starting engine: %s",
                 err.value().c_str());
    response->success = false;
    response->message = err.value();
    return;
  }

  response->success = true;
  requested_run_ = true;
}

//==============================================================================
void Engine::stop_engine_callback(
    const std::shared_ptr<StopEngineSrv::Request> request,
    std::shared_ptr<StopEngineSrv::Response> response) {
  const auto task_it =
    std::find_if(this->tasks_.begin(), this->tasks_.end(), [](const auto& it) {
      if (it.second.status == TaskStatus::STARTED)
        return true;
      return false;
  });
  if (task_it == this->tasks_.end()) {
    const std::string error = "Failed stopping engine, engine not started";
    RCLCPP_ERROR(node_->get_logger(), "%s", error.c_str());
    response->success = false;
    response->message = error;
    return;
  }

  task_it->second.score.tier_1_success();
  task_it->second.status = TaskStatus::FINISHED;
  compute_score(task_it->second.score);

  if (request->finished) {
    this->score_run();
    this->reset_engine();
  }

  this->requested_run_ = false;
  response->success = true;
}

//==============================================================================
std::optional<std::string> Engine::initialize() {
  RCLCPP_INFO(node_->get_logger(),
              "╔════════════════════════════════════════╗");
  RCLCPP_INFO(node_->get_logger(),
              "║   Initializing AIC Engine...           ║");
  RCLCPP_INFO(node_->get_logger(),
              "╚════════════════════════════════════════╝");

  // Make sure a valid clock is received, it takes time to initialize
  // the subscriber and following timeout calls might fail otherwise
  RCLCPP_INFO(node_->get_logger(), "Waiting for clock");
  if (!node_->get_clock()->wait_until_started(rclcpp::Duration(10, 0))) {
    const std::string error = "Failed to find a valid clock";
    return error;
  }
  RCLCPP_INFO(node_->get_logger(), "Clock found successfully.");

  // Create output directory for bag files.
  std::error_code ec;
  std::filesystem::create_directories(scoring_output_dir_, ec);
  if (ec) {
    const std::string error = "Failed to create bag output directory '" +
                              scoring_output_dir_ +
                              "': " + ec.message().c_str();
    return error;
  }
  RCLCPP_INFO(node_->get_logger(), "Bag output directory: %s",
              scoring_output_dir_.c_str());

  constexpr int MAX_RETRIES = 5;

  if (!this->check_model()) {
    const std::string error = "  ✗ Participant model is not ready.";
    return error;
  }

  RCLCPP_INFO(node_->get_logger(), "  ✓ Model Ready");

  bool success = false;
  for (int attempt = 1; attempt <= MAX_RETRIES && !success; ++attempt) {
    if (attempt > 1) {
      RCLCPP_WARN(node_->get_logger(),
                  "  ⟳ Retrying check_endpoints (attempt %d/%d)...", attempt,
                  MAX_RETRIES);
    }
    if (this->check_endpoints()) {
      success = true;
    }
  }
  if (!success) {
    const std::string error =
        "  ✗ EVALUATION ERROR: Endpoints check failed "
        "after " +
        std::to_string(MAX_RETRIES) +
        " attempts. "
        "This is an infrastructure issue. Is eval environment "
        "started?";
    return error;
  }

  RCLCPP_INFO(node_->get_logger(), "  ✓ Endpoints Ready ");
  RCLCPP_INFO(node_->get_logger(), "✓ AIC Engine initialized successfully!");

  return std::nullopt;
}

//==============================================================================
void Engine::reset_engine() {
  if (this->scoring_tier2_->IsRecording()) {
    this->scoring_tier2_->StopRecording();
  }
  this->tasks_.clear();
  this->requested_run_ = false;

  // Force deactivation + cleanup to make sure we are in a clean state
  this->deactivate_model_node();
  this->cleanup_model_node();
}

//==============================================================================
std::optional<std::string> Engine::start_trial(const TaskMsg& task) {
  RCLCPP_INFO(node_->get_logger(), " ");
  RCLCPP_INFO(node_->get_logger(),
              "╔════════════════════════════════════════╗");
  RCLCPP_INFO(node_->get_logger(),
              "║      Starting AIC Engine Trial         ║");
  RCLCPP_INFO(node_->get_logger(),
              "╚════════════════════════════════════════╝");
  RCLCPP_INFO(node_->get_logger(), " ");

  if (!this->ready_scoring(task)) {
    const std::string error =
        "  ✗ EVALUATION ERROR: Scoring setup failed."
        "This is an infrastructure issue. Is eval environment "
        "started?";
    return error;
  }

  RCLCPP_INFO(node_->get_logger(), "  ✓ Scoring Ready");

  this->scoring_tier2_->SetTaskStartTime(this->node_->now());
  this->tasks_.insert({task.id, task});
  return std::nullopt;
}

/// Given a set [s1, s2, s3] returns a string "s1, s2, s3"
//==============================================================================
static std::string string_set_to_csv(const std::set<std::string>& strings) {
  if (strings.empty()) {
    return "";
  }
  auto it = strings.begin();
  std::string result;
  for (; it != std::prev(strings.end()); ++it) {
    result += *it + ", ";
  }
  result += *it;
  return result;
}

//==============================================================================
bool Engine::model_node_is_unconfigured() {
  RCLCPP_INFO(node_->get_logger(),
              "Lifecycle node '%s' is available. Checking if it is in "
              "'unconfigured' state...",
              model_node_name_.c_str());

  if (!model_get_state_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "GetState service '%s' not available after waiting",
                 model_get_state_service_name_.c_str());
    return false;
  }

  // Call the service to get current state
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto future = model_get_state_client_->async_send_request(request);

  if (!wait_for_interruptible(future, std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "GetState service call timed out for node '%s'",
                 model_node_name_.c_str());
    return false;
  }

  auto response = future.get();

  // Check if the state is unconfigured (PRIMARY_STATE_UNCONFIGURED = 1)
  if (response->current_state.id !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Lifecycle node '%s' is not in 'unconfigured' state. Current "
                 "state: %s (id: %u)",
                 model_node_name_.c_str(),
                 response->current_state.label.c_str(),
                 response->current_state.id);
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
              "Lifecycle node '%s' is in 'unconfigured' state",
              model_node_name_.c_str());

  return true;
}

//==============================================================================
bool Engine::configure_model_node() {
  RCLCPP_INFO(node_->get_logger(), "Configuring lifecycle node '%s'...",
              model_node_name_.c_str());

  if (!this->transition_model_lifecycle_node(
          lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
    return false;
  }

  // Check that the model rejects action goals.
  if (!insert_cable_action_client_->wait_for_action_server(
          std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Insert cable action server not available after waiting");
    return false;
  }
  auto goal_was_rejected = std::make_shared<bool>(false);
  auto goal_msg = InsertCableAction::Goal();
  auto goal_options =
      rclcpp_action::Client<InsertCableAction>::SendGoalOptions();
  goal_options
      .goal_response_callback = [this, goal_was_rejected](
                                    const rclcpp_action::ClientGoalHandle<
                                        InsertCableAction>::SharedPtr&
                                        goal_handle) {
    if (!goal_handle) {
      RCLCPP_INFO(
          this->node_->get_logger(),
          "Insert cable action goal was rejected by the server as expected.");
      *goal_was_rejected = true;
    } else {
      RCLCPP_ERROR(this->node_->get_logger(),
                   "Insert cable action goal was accepted by the server while "
                   "in 'configured' state. This is a rule violation.");
    }
  };

  insert_cable_action_client_->async_send_goal(goal_msg, goal_options);
  node_->get_clock()->sleep_for(rclcpp::Duration(std::chrono::seconds(1)));

  if (!*goal_was_rejected) {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
              "Lifecycle node '%s' is in 'configured' state and meets all "
              "expectations.",
              model_node_name_.c_str());

  return true;
}

//==============================================================================
bool Engine::check_model() {
  RCLCPP_INFO(node_->get_logger(), "Checking participant model readiness...");

  if (skip_model_ready_) {
    RCLCPP_WARN(node_->get_logger(),
                "Skipping model readiness check as per parameter.");
    return true;
  }

  rclcpp::Time start_time = this->node_->now();
  const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(
      this->node_->get_parameter("model_discovery_timeout_seconds").as_int());

  // Check if aic_model node exists in the graph and is a lifecycle node.
  bool model_discovered = false;

  while (rclcpp::ok() && !model_discovered &&
         !(this->node_->now() - start_time > timeout)) {
    // First check that only one node with the expected name exists.
    auto node_graph = node_->get_node_graph_interface();
    auto node_names_and_namespaces =
        node_graph->get_node_names_and_namespaces();
    int model_node_count = 0;
    for (const auto& [name, namespace_] : node_names_and_namespaces) {
      if (name == model_node_name_) {
        model_node_count++;
      }
    }
    if (model_node_count > 1) {
      RCLCPP_ERROR(node_->get_logger(),
                   "More than one node with name '%s' found",
                   model_node_name_.c_str());
      return false;
    }
    if (model_node_count == 0) {
      RCLCPP_INFO(node_->get_logger(),
                  "No node with name '%s' found. Retrying...",
                  model_node_name_.c_str());
      node_->get_clock()->sleep_for(
          rclcpp::Duration(std::chrono::milliseconds(1000)));
      continue;
    }

    // Now ensure that the get_state service exists and is of the correct type.
    RCLCPP_INFO(node_->get_logger(),
                "Found %d node(s) with name '%s'. Checking if it is a "
                "lifecycle node...",
                model_node_count, model_node_name_.c_str());
    if (!model_get_state_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_INFO(node_->get_logger(),
                  "Service '%s' not available yet. Retrying...",
                  model_get_state_service_name_.c_str());
      node_->get_clock()->sleep_for(
          rclcpp::Duration(std::chrono::milliseconds(1000)));
      continue;
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "Service '%s' is available. Participant model discovered.",
                  model_get_state_service_name_.c_str());
      model_discovered = true;
      break;
    }
  }

  if (!model_discovered) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Lifecycle node '%s' not discovered after waiting (checked "
                 "for service '%s' with type 'lifecycle_msgs/srv/GetState')",
                 model_node_name_.c_str(),
                 model_get_state_service_name_.c_str());
    return false;
  }

  if (!model_node_is_unconfigured()) {
    return false;
  }

  if (!configure_model_node()) {
    return false;
  }

  // Activate the model node
  if (!activate_model_node()) {
    return false;
  }

  return true;
}

//==============================================================================
bool Engine::check_endpoints() {
  RCLCPP_INFO(node_->get_logger(), "Checking required endpoints...");
  return true;

  // Check nodes
  std::set<std::string> unavailable = {this->adapter_node_name_};
  rclcpp::Time start_time = this->node_->now();
  const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(
      this->node_->get_parameter("endpoint_ready_timeout_seconds").as_int());
  const auto& node_graph = node_->get_node_graph_interface();

  while (rclcpp::ok() && !unavailable.empty() &&
         !(this->node_->now() - start_time > timeout)) {
    std::unordered_set<std::string> node_set;
    for (const auto& [name, _] : node_graph->get_node_names_and_namespaces()) {
      node_set.insert(name);
    }
    for (auto it = unavailable.begin(); it != unavailable.end();) {
      if (node_set.count(*it)) {
        // Node found, remove it from unavailable list
        it = unavailable.erase(it);
      } else {
        ++it;
      }
    }
    node_->get_clock()->sleep_for(
        rclcpp::Duration(std::chrono::milliseconds(10)));
  }
  if (!unavailable.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required nodes: %s",
                 string_set_to_csv(unavailable).c_str());
    return false;
  }

  // Check topics
  // TODO(Yadunund): Consider checking for messages received on topics.
  unavailable = this->scoring_tier2_->GetMissingRequiredTopics();
  start_time = this->node_->now();
  while (rclcpp::ok() && !unavailable.empty() &&
         !(this->node_->now() - start_time > timeout)) {
    node_->get_clock()->sleep_for(
        rclcpp::Duration(std::chrono::milliseconds(10)));
    unavailable = this->scoring_tier2_->GetMissingRequiredTopics();
  }
  if (!unavailable.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required topics: %s",
                 string_set_to_csv(unavailable).c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "All required endpoints are available.");
  return true;
}

//==============================================================================
bool Engine::ready_scoring(const TaskMsg& task) {
  RCLCPP_INFO(node_->get_logger(), "Checking scoring system readiness...");
  // Register the connection for this trial.
  aic_scoring::Connection connection;
  connection.cableName = task.cable_name;
  connection.plugName = task.plug_name;
  connection.portName = task.port_name;
  connection.targetModuleName = task.target_module_name;

  // Create unique bag filename with timestamp
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) %
            1000;

  std::ostringstream oss;
  oss << scoring_output_dir_ << "/bag_" << "_"
      << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_"
      << std::setfill('0') << std::setw(3) << ms.count();
  const std::string bag_path = oss.str();

  // Add a few seconds for safety since this is a limit for recorded data
  const unsigned int max_task_limit = task.time_limit + 10;
  if (!scoring_tier2_->StartRecording(bag_path, connection,
                                      std::chrono::seconds(max_task_limit))) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to start recording to '%s'.",
                 bag_path.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Started recording to '%s'.",
              bag_path.c_str());
  return true;
}

//==============================================================================
bool Engine::transition_model_lifecycle_node(const uint8_t transition) {
  std::string transition_name;
  switch (transition) {
    case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE:
      transition_name = "configure";
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE:
      transition_name = "activate";
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE:
      transition_name = "deactivate";
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP:
      transition_name = "cleanup";
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN:
      [[fallthrough]];
    case lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN:
      [[fallthrough]];
    case lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN:
      transition_name = "shutdown";
      break;
    default:
      RCLCPP_ERROR(
          node_->get_logger(),
          "Failed to transition model node, transition %u not recognized",
          (int)transition);
      return false;
  }
  const std::string timeout_param_name =
      "model_" + transition_name + "_timeout_seconds";
  const int timeout = this->node_->get_parameter(timeout_param_name).as_int();

  RCLCPP_INFO(node_->get_logger(),
              "Transitioning model node '%s' to transition '%s'...",
              model_node_name_.c_str(), transition_name.c_str());

  auto change_state_request =
      std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  change_state_request->transition.id = transition;

  auto future =
      model_change_state_client_->async_send_request(change_state_request);
  if (!wait_for_interruptible(future, std::chrono::seconds(timeout))) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "ChangeState service call timed out for transition '%s' for node '%s'",
        transition_name.c_str(), model_node_name_.c_str());
    return false;
  }

  auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to transition model node '%s' to state '%s'",
                 model_node_name_.c_str(), transition_name.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
              "Successfully transition model node '%s' to state '%s'",
              model_node_name_.c_str(), transition_name.c_str());

  return true;
}

//==============================================================================
bool Engine::activate_model_node() {
  if (skip_model_ready_) {
    RCLCPP_INFO(node_->get_logger(),
                "Skipping model activation as per parameter.");
    return true;
  }

  // TODO(Yadunund): Verify active requirements.
  return this->transition_model_lifecycle_node(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}

//==============================================================================
bool Engine::deactivate_model_node() {
  if (skip_model_ready_) {
    RCLCPP_INFO(node_->get_logger(),
                "Skipping model deactivation as per parameter.");
    return true;
  }
  return this->transition_model_lifecycle_node(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
}

//==============================================================================
bool Engine::cleanup_model_node() {
  if (skip_model_ready_) {
    RCLCPP_INFO(node_->get_logger(),
                "Skipping model cleanup as per parameter.");
    return true;
  }

  return this->transition_model_lifecycle_node(
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
}

//==============================================================================
void Engine::compute_score(TrialScore& score) {
  if (!scoring_tier2_->StopRecording()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to stop recording.");
    return;
  }
  auto [tier2_score, tier3_score] = scoring_tier2_->ComputeScore();
  score.tier_2 = tier2_score;
  score.tier_3 = tier3_score;

  RCLCPP_INFO(node_->get_logger(), "Finished scoring trial, total score is: %f",
              score.total_score());
}

//==============================================================================
void Engine::score_run() {
  YAML::Node score;
  double total_score = 0;
  for (const auto& [trial_name, attempt] : this->tasks_) {
    score[trial_name]["tier_1"] = attempt.score.tier_1.to_yaml();
    score[trial_name]["tier_2"] = attempt.score.tier_2.to_yaml();
    score[trial_name]["tier_3"] = attempt.score.tier_3.to_yaml();
    total_score += attempt.score.tier_1.total_score();
    total_score += attempt.score.tier_2.total_score();
    total_score += attempt.score.tier_3.total_score();
  }
  score["total"] = total_score;

  const std::string yaml_output_file = scoring_output_dir_ + "/scoring.yaml";
  std::ofstream fout(yaml_output_file);
  fout << score;

  std::stringstream ss;
  ss << score;
  RCLCPP_INFO(node_->get_logger(),
              "╔════════════════════════════════════════╗");
  RCLCPP_INFO(node_->get_logger(),
              "║        Complete Scoring Results        ║");
  RCLCPP_INFO(node_->get_logger(),
              "╚════════════════════════════════════════╝");

  // Split the YAML output by lines and print each line
  std::string line;
  while (std::getline(ss, line)) {
    RCLCPP_INFO(node_->get_logger(), "%s", line.c_str());
  }
  RCLCPP_INFO(node_->get_logger(), " ");
}

}  // namespace aic
