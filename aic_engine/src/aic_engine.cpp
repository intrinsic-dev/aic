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
#include "google/longrunning/operations.pb.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/subscription_options.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace aic {

//==============================================================================
YAML::Node Score::serialize() const {
  const double total_score = this->calculate_total_score();
  YAML::Node score;
  score["total"] = total_score;
  score["milestone_0"] = this->tier_1.to_yaml();
  for (const auto& [trial_name, trial_score] : this->breakdown) {
    score[trial_name] = trial_score.to_yaml();
  }
  return score;
}

//==============================================================================
double Score::calculate_total_score() const {
  double score = this->tier_1.total_score();
  for (const auto& [trial_name, trial_score] : this->breakdown) {
    score += trial_score.total_score();
  }
  return score;
}

//==============================================================================
Engine::Engine(const rclcpp::NodeOptions& options)
    : node_(std::make_shared<rclcpp::Node>("aic_engine", options)),
      insert_cable_action_client_(nullptr),
      model_discovered_(false) {
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
  ground_truth_ = node_->declare_parameter("ground_truth", false);
  skip_model_ready_ = node_->declare_parameter("skip_model_ready", false);
  node_->declare_parameter("model_discovery_timeout_seconds", 30);
  node_->declare_parameter("model_configure_timeout_seconds", 60);
  node_->declare_parameter("model_activate_timeout_seconds", 60);
  node_->declare_parameter("model_deactivate_timeout_seconds", 60);
  node_->declare_parameter("model_cleanup_timeout_seconds", 60);
  node_->declare_parameter("model_shutdown_timeout_seconds", 60);
  executive_service_address_ = node_->declare_parameter(
      "executive_service_address", std::string("localhost:17080"));

  // Create executive service stub.
  auto channel = grpc::CreateChannel(executive_service_address_,
                                     grpc::InsecureChannelCredentials());
  executive_stub_ = ExecutiveService::NewStub(channel);

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

  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  start_engine_server_ = node_->create_service<StartEngineSrv>("/start_aic_engine",
      std::bind(&Engine::start_engine_callback, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);

  spin_thread_ = std::thread([node = node_]() {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  });
}

//==============================================================================
void Engine::start_engine_callback(const std::shared_ptr<StartEngineSrv::Request> request,
    std::shared_ptr<StartEngineSrv::Response> response)
{
  if (requested_run_) {
    const std::string error = "Failed starting engine, a run is already underway";
    RCLCPP_ERROR(node_->get_logger(), "%s", error.c_str());
    response->success = false;
    response->message = error;
    return;
  }

  const auto initialization_result = this->initialize(request->config);
  if (initialization_result.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed initializing engine: %s", initialization_result.value().c_str());
    response->success = false;
    response->message = initialization_result.value();
    return;
  }

  const auto err = this->run();
  if (err.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed starting engine: %s", err.value().c_str());
    response->success = false;
    response->message = err.value();
    return;
  }

  response->success = true;
  requested_run_ = true;
}

//==============================================================================
std::optional<std::string> Engine::initialize(const std::string& yaml_config) {
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;36m╔════════════════════════════════════════╗\033[0m");
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;36m║   Initializing AIC Engine...           ║\033[0m");
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;36m╚════════════════════════════════════════╝\033[0m");

  // Try to load config as YAML
  try {
    config_ = YAML::Load(yaml_config);
  } catch (const YAML::Exception& e) {
    const std::string error = "Failed to load config '" + yaml_config + "': " + e.what();
    return error;
  } catch (const std::exception& e) {
    const std::string error = "Failed to load config '" + yaml_config + "': " + e.what();
    return error;
  }

  // Make sure a valid clock is received, it takes time to initialize
  // the subscriber and following timeout calls might fail otherwise
  RCLCPP_INFO(node_->get_logger(), "Waiting for clock");
  if (!node_->get_clock()->wait_until_started(rclcpp::Duration(10, 0))) {
    const std::string error = "Failed to find a valid clock";
    return error;
  }
  RCLCPP_INFO(node_->get_logger(), "Clock found successfully.");

  // Create ROS endpoints.
  const rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = callback_group_;

  joint_motion_update_sub_ = node_->create_subscription<JointMotionUpdateMsg>(
      "/aic_controller/joint_motion_update", reliable_qos,
      [this](JointMotionUpdateMsg::ConstSharedPtr msg) {
        last_joint_motion_update_msg_ = msg;
      }, sub_options);
  motion_update_sub_ = node_->create_subscription<MotionUpdateMsg>(
      "/aic_controller/motion_update", reliable_qos,
      [this](MotionUpdateMsg::ConstSharedPtr msg) {
        last_motion_update_msg_ = msg;
      }, sub_options);

  insert_cable_action_client_ =
      rclcpp_action::create_client<InsertCableAction>(node_, "/insert_cable");
  model_get_state_client_ = node_->create_client<lifecycle_msgs::srv::GetState>(
      model_get_state_service_name_, rclcpp::ServicesQoS(), callback_group_);
  model_change_state_client_ =
      node_->create_client<lifecycle_msgs::srv::ChangeState>(
          model_change_state_service_name_, rclcpp::ServicesQoS(), callback_group_);
  tare_ft_client_ = node_->create_client<TriggerSrv>(
      "/aic_controller/tare_force_torque_sensor", rclcpp::ServicesQoS(), callback_group_);

  scoring_tier2_ = std::make_unique<aic_scoring::ScoringTier2>(node_.get());
  // TODO(luca) Change initialization to static topic names / types?
  /*
  if (!scoring_tier2_->Initialize(config_["scoring"])) {
    const std::string error = "Failed to initialize scoring system";
    return error;
  }
  */
  scoring_tier2_->SetGripperFrame(
      node_->get_parameter("gripper_frame_name").as_string());

  // Create output directory for bag files.
  std::error_code ec;
  std::filesystem::create_directories(scoring_output_dir_, ec);
  if (ec) {
    const std::string error = "Failed to create bag output directory '" + scoring_output_dir_ + "': " + ec.message().c_str();
    return error;
  }
  RCLCPP_INFO(node_->get_logger(), "Bag output directory: %s",
              scoring_output_dir_.c_str());

  RCLCPP_INFO(node_->get_logger(),
              "\033[1;32m✓ AIC Engine initialized successfully!\033[0m");

  return std::nullopt;
}

//==============================================================================
std::optional<std::string> Engine::run() {
  RCLCPP_INFO(node_->get_logger(), " ");
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;35m╔════════════════════════════════════════╗\033[0m");
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;35m║      Starting AIC Engine Run           ║\033[0m");
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;35m╚════════════════════════════════════════╝\033[0m");
  RCLCPP_INFO(node_->get_logger(), " ");

  TrialResult trial_result = this->handle_trial();

  RCLCPP_INFO(node_->get_logger(), " ");
  if (trial_result.error.has_value()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "\033[1;31m╔════════════════════════════════════════╗\033[0m");
    RCLCPP_ERROR(node_->get_logger(),
                 "\033[1;31m║   ✗ Engine Stopped with Errors         ║\033[0m");
    RCLCPP_ERROR(node_->get_logger(),
                 "\033[1;31m╚════════════════════════════════════════╝\033[0m");
    this->score_run(trial_result.score);
    this->reset_after_trial();
  }
  return trial_result.error;
}

//==============================================================================
void Engine::process() {
  while (rclcpp::ok()) {
    // Wait for an incoming request. Pretty lightweight so just normal spinlock
    while (!this->requested_run_) {
      if (!rclcpp::ok())
        return;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(node_->get_logger(), "Received request for scoring, starting...");

    if (this->tasks_completed_successfully()) {
      RCLCPP_INFO(node_->get_logger(),
                  "\033[1;32m  ✓ Tasks Completed!\033[0m");
    } else {
      RCLCPP_ERROR(node_->get_logger(),
                   "\033[1;31m  ✗ Tasks were not completed successfully.\033[0m");
    }
    Score score;
    score.tier_1_success();
    compute_score(score);
    this->score_run(score);
    reset_after_trial();
    requested_run_ = false;
  }
}

//==============================================================================
TrialResult Engine::handle_trial() {
  Score score;

  constexpr int MAX_RETRIES = 5;

  if (!this->check_model()) {
    const std::string error =
        "\033[1;31m  ✗ Participant model is not ready.'\033[0m";
    return {score, error};
  }

  RCLCPP_INFO(node_->get_logger(),
              "\033[1;32m  ✓ Model Ready\033[0m");
  score.tier_1_success();

  bool success = false;
  for (int attempt = 1; attempt <= MAX_RETRIES && !success; ++attempt) {
    if (attempt > 1) {
      RCLCPP_WARN(node_->get_logger(),
                  "\033[1;33m  ⟳ Retrying check_endpoints (attempt %d/%d)...\033[0m",
                  attempt, MAX_RETRIES);
    }
    if (this->check_endpoints()) {
      success = true;
    }
  }
  if (!success) {
    const std::string error =
                 "\033[1;31m  ✗ EVALUATION ERROR: Endpoints check failed "
                 "after " + std::to_string(MAX_RETRIES) + " attempts. "
                 "This is an infrastructure issue. Is eval environment "
                 "started?\033[0m";
    return {score, error};
  }

  RCLCPP_INFO(node_->get_logger(), "\033[1;32m  ✓ Endpoints Ready \033[0m");

  success = false;
  for (int attempt = 1; attempt <= MAX_RETRIES && !success; ++attempt) {
    if (attempt > 1) {
      RCLCPP_WARN(node_->get_logger(),
                  "\033[1;33m  ⟳ Retrying ready_scoring (attempt %d/%d)...\033[0m",
                  attempt, MAX_RETRIES);
    }
    if (this->ready_scoring()) {
      success = true;
    }
  }
  if (!success) {
    const std::string error =
                 "\033[1;31m  ✗ EVALUATION ERROR: Scoring setup failed "
                 "after " + std::to_string(MAX_RETRIES) + " attempts. "
                 "This is an infrastructure issue. Is eval environment "
                 "started?\033[0m";
    return {score, error};
  }


  RCLCPP_INFO(node_->get_logger(),
              "\033[1;32m  ✓ Scoring Ready\033[0m");

  if (!this->tasks_started()) {
    const std::string error = "\033[1;31m  ✗ Tasks not found.\033[0m";
    return {score, error};
  }

  RCLCPP_INFO(node_->get_logger(),
              "\033[1;36m  ⟳ Tasks Executing...\033[0m");
  this->scoring_tier2_->SetTaskStartTime(this->node_->now());
  return {score};
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
bool Engine::model_node_moved_robot() {
  // TODO(Yadunund): We'll need to make this check more effective.
  // The model could always publish this after we check here.
  if (last_joint_motion_update_msg_ != nullptr ||
      last_motion_update_msg_ != nullptr) {
    return true;
  }
  return false;
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

  // Check that the model is not publishing any robot command topics.
  if (model_node_moved_robot()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Participant model is publishing command topics "
                 "while in 'unconfigured' state. This is a rule violation.");
    return false;
  }

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

  if (model_node_moved_robot()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Participant model is publishing command topics "
                 "while in 'configured' state. This is a rule violation.");
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
  model_discovered_ = false;

  while (rclcpp::ok() && !model_discovered_ &&
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
      model_discovered_ = true;
      break;
    }
  }

  if (!model_discovered_) {
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
bool Engine::ready_scoring() {
  RCLCPP_INFO(node_->get_logger(), "Checking scoring system readiness...");
  return true;
  // Register the new connections for this trial.
  std::vector<aic_scoring::Connection> connections;
  // TODO(luca) Register the vector of connections
  /*
  for (const auto& task : trial.tasks) {
    aic_scoring::Connection connection;
    connection.cableName = task.cable_name;
    connection.taskBoardName = "task_board";
    connection.plugName = task.plug_name;
    connection.portName = task.port_name;
    connection.targetModuleName = task.target_module_name;
    connections.push_back(connection);
  }
  */

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

  unsigned int max_task_limit = 0;
  // TODO(luca) A const for time limit? Or just get rid of it?
  /*
  for (const auto& task : trial.tasks) {
    if (task.time_limit > max_task_limit) {
      max_task_limit = task.time_limit;
    }
  }
  */
  // Add a few seconds for safety since this is a limit for recorded data
  max_task_limit += 5;
  if (!scoring_tier2_->StartRecording(bag_path, connections,
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
bool Engine::tasks_started() {
  // Get current operation name.
  google::longrunning::ListOperationsRequest list_request;
  google::longrunning::ListOperationsResponse list_response;
  grpc::ClientContext context;
  auto status = executive_stub_->ListOperations(&context, list_request, &list_response);
  current_operation_name_ = "";
  if (status.ok()) {
    for (int i = 0; i < list_response.operations_size(); ++i) {
      if (!list_response.operations(i).done()) {
        current_operation_name_ = list_response.operations(i).name();
        RCLCPP_INFO(node_->get_logger(), "Found running operation: %s",
                    current_operation_name_.c_str());
        break;
      }
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to list operations: %s",
                 status.error_message().c_str());
    return false;
  }
  if (current_operation_name_ == "") {
    RCLCPP_ERROR(node_->get_logger(), "No running operation found");
    return false;
  }
  return true;
}

//==============================================================================
bool Engine::tasks_completed_successfully() {
  RCLCPP_INFO(node_->get_logger(),
              "Checking if all tasks were completed successfully...");

  // Wait until long running operation is done here
  if (!current_operation_name_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for operation %s to complete...",
                current_operation_name_.c_str());
    google::longrunning::WaitOperationRequest wait_request;
    wait_request.set_name(current_operation_name_);
    google::longrunning::Operation operation;
    grpc::ClientContext wait_context;
    auto wait_status =
        executive_stub_->WaitOperation(&wait_context, wait_request, &operation);
    current_operation_name_ = "";
    if (!wait_status.ok()) {
      RCLCPP_ERROR(node_->get_logger(), "Error waiting for operation: %s",
                   wait_status.error_message().c_str());
      return false;
    } else {
      RCLCPP_INFO(node_->get_logger(), "Operation %s completed.",
                  current_operation_name_.c_str());
      this->scoring_tier2_->SetTaskEndTime(this->node_->now());
      return true;
    }
  }
  return false;
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

  if (!this->model_discovered_) {
    return true;
  }

  return this->transition_model_lifecycle_node(
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
}

//==============================================================================
void Engine::reset_after_trial() {
  RCLCPP_INFO(node_->get_logger(), "Resetting after trial completion...");

  // Deactivate the model node to transition back to configured state
  if (this->model_discovered_) {
    this->deactivate_model_node();
    this->cleanup_model_node();
  }

  RCLCPP_INFO(node_->get_logger(), "Reset after trial completed.");
}

//==============================================================================
Engine::~Engine() {
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

//==============================================================================
void Engine::compute_score(Score& score) {
  if (!scoring_tier2_->StopRecording()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to stop recording.");
    return;
  }
  auto [tier2_score, tier3_score] = scoring_tier2_->ComputeScore();
  // TODO(luca) Change return type of scoring and fill Score here
  /*
  score.tier_2 = tier2_score;
  score.tier_3 = tier3_score;
  */

  RCLCPP_INFO(node_->get_logger(), "Finished scoring run, total score is: %f",
              score.calculate_total_score());
}

//==============================================================================
void Engine::score_run(const Score& score) {
  YAML::Node node = score.serialize();

  const std::string yaml_output_file = scoring_output_dir_ + "/scoring.yaml";
  std::ofstream fout(yaml_output_file);
  fout << node;

  std::stringstream ss;
  ss << node;
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;36m╔════════════════════════════════════════╗\033[0m");
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;36m║        Complete Scoring Results        ║\033[0m");
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;36m╚════════════════════════════════════════╝\033[0m");

  // Split the YAML output by lines and print each line
  std::string line;
  while (std::getline(ss, line)) {
    RCLCPP_INFO(node_->get_logger(), "\033[1;36m%s\033[0m", line.c_str());
  }
  RCLCPP_INFO(node_->get_logger(), " ");
}

}  // namespace aic
