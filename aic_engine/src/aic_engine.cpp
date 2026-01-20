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

#include <cmath>
#include <filesystem>
#include <sstream>
#include <unordered_set>

#include "aic_task_interfaces/msg/task.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/subscription_options.hpp"

namespace aic {

//==============================================================================
Trial::Trial(const std::string& _id, YAML::Node _config) : id(std::move(_id)) {
  // Validate config structure
  if (!_config["scene"]) {
    throw std::runtime_error("Config missing required key: 'scene'");
  }
  if (!_config["tasks"]) {
    throw std::runtime_error("Config missing required key: 'tasks'");
  }

  const auto& scene = _config["scene"];

  // Validate scene.task_board
  if (!scene["task_board"]) {
    throw std::runtime_error("Config missing required key: 'scene.task_board'");
  }
  const auto& task_board = scene["task_board"];
  if (!task_board["pose"]) {
    throw std::runtime_error(
        "Config missing required key: 'scene.task_board.pose'");
  }
  const auto& task_board_pose = task_board["pose"];
  for (const auto& key : {"x", "y", "z", "roll", "pitch", "yaw"}) {
    if (!task_board_pose[key]) {
      throw std::runtime_error(
          std::string("Config missing required key: 'scene.task_board.pose.") +
          key + "'");
    }
  }

  // Validate NIC rails (nic_rail_0 through nic_rail_4)
  for (int i = 0; i < 5; ++i) {
    std::string rail_key = "nic_rail_" + std::to_string(i);
    if (!task_board[rail_key]) {
      throw std::runtime_error(
          "Config missing required key: 'scene.task_board." + rail_key + "'");
    }
    const auto& rail = task_board[rail_key];
    if (!rail["entity_present"]) {
      throw std::runtime_error(
          "Config missing required key: 'scene.task_board." + rail_key +
          ".entity_present'");
    }
    if (rail["entity_present"].as<bool>()) {
      if (!rail["entity_name"]) {
        throw std::runtime_error(
            "Config missing required key: 'scene.task_board." + rail_key +
            ".entity_name'");
      }
      if (!rail["entity_pose"]) {
        throw std::runtime_error(
            "Config missing required key: 'scene.task_board." + rail_key +
            ".entity_pose'");
      }
      const auto& entity_pose = rail["entity_pose"];
      for (const auto& key : {"translation", "roll", "pitch", "yaw"}) {
        if (!entity_pose[key]) {
          throw std::runtime_error(
              "Config missing required key: 'scene.task_board." + rail_key +
              ".entity_pose." + key + "'");
        }
      }
    }
  }

  // Validate SC rails (sc_rail_0 and sc_rail_1)
  for (int i = 0; i < 2; ++i) {
    std::string rail_key = "sc_rail_" + std::to_string(i);
    if (!task_board[rail_key]) {
      throw std::runtime_error(
          "Config missing required key: 'scene.task_board." + rail_key + "'");
    }
    const auto& rail = task_board[rail_key];
    if (!rail["entity_present"]) {
      throw std::runtime_error(
          "Config missing required key: 'scene.task_board." + rail_key +
          ".entity_present'");
    }
    if (rail["entity_present"].as<bool>()) {
      if (!rail["entity_name"]) {
        throw std::runtime_error(
            "Config missing required key: 'scene.task_board." + rail_key +
            ".entity_name'");
      }
      if (!rail["entity_pose"]) {
        throw std::runtime_error(
            "Config missing required key: 'scene.task_board." + rail_key +
            ".entity_pose'");
      }
      const auto& entity_pose = rail["entity_pose"];
      for (const auto& key : {"translation", "roll", "pitch", "yaw"}) {
        if (!entity_pose[key]) {
          throw std::runtime_error(
              "Config missing required key: 'scene.task_board." + rail_key +
              ".entity_pose." + key + "'");
        }
      }
    }
  }

  // Validate rail structure if rails exist
  for (int i = 0; i < 6; ++i) {
    std::string rail_key = "rail_" + std::to_string(i);
    if (task_board[rail_key]) {
      // If rail exists and has ports, validate port structure
      const auto& rail = task_board[rail_key];
      if (rail["ports"]) {
        const auto& ports = rail["ports"];
        for (auto it = ports.begin(); it != ports.end(); ++it) {
          const auto& port = it->second;
          if (!port["type"]) {
            throw std::runtime_error(
                "Config missing required key: 'scene.task_board." + rail_key +
                ".ports." + it->first.as<std::string>() + ".type'");
          }
          if (!port["entity_name"]) {
            throw std::runtime_error(
                "Config missing required key: 'scene.task_board." + rail_key +
                ".ports." + it->first.as<std::string>() + ".entity_name'");
          }
          if (!port["entity_pose"]) {
            throw std::runtime_error(
                "Config missing required key: 'scene.task_board." + rail_key +
                ".ports." + it->first.as<std::string>() + ".entity_pose'");
          }
          const auto& entity_pose = port["entity_pose"];
          for (const auto& key : {"translation", "roll", "pitch", "yaw"}) {
            if (!entity_pose[key]) {
              throw std::runtime_error(
                  "Config missing required key: 'scene.task_board." + rail_key +
                  ".ports." + it->first.as<std::string>() + ".entity_pose." +
                  key + "'");
            }
          }
        }
      }
    }
  }

  // Validate scene.cable
  if (!scene["cable"]) {
    throw std::runtime_error("Config missing required key: 'scene.cable'");
  }
  const auto& cable = scene["cable"];
  if (!cable["pose"]) {
    throw std::runtime_error("Config missing required key: 'scene.cable.pose'");
  }
  const auto& cable_pose = cable["pose"];
  for (const auto& key : {"gripper_offset", "roll", "pitch", "yaw"}) {
    if (!cable_pose[key]) {
      throw std::runtime_error(
          std::string("Config missing required key: 'scene.cable.pose.") + key +
          "'");
    }
  }
  const auto& cable_pose_offset = cable["pose"]["gripper_offset"];
  for (const auto& key : {"x", "y", "z"}) {
    if (!cable_pose_offset[key]) {
      throw std::runtime_error(
          std::string("Config missing required key: "
                      "'scene.cable.pose.gripper_offset.") +
          key + "'");
    }
  }
  if (!cable["attach_cable_to_gripper"]) {
    throw std::runtime_error(
        "Config missing required key: 'scene.cable.attach_cable_to_gripper'");
  }
  if (!cable["cable_type"]) {
    throw std::runtime_error(
        "Config missing required key: 'scene.cable.cable_type'");
  }

  // Validate tasks array
  const auto& tasks = _config["tasks"];
  if (!tasks.IsMap() || tasks.size() == 0) {
    throw std::runtime_error("Config 'tasks' must be a non-empty dictionary");
  }

  // Validate and parse all tasks
  for (auto it = tasks.begin(); it != tasks.end(); ++it) {
    const std::string task_id = it->first.as<std::string>();
    const YAML::Node task_config = it->second;
    for (const auto& key :
         {"cable_type", "cable_name", "plug_type", "plug_name", "port_type",
          "port_name", "target_module_name", "time_limit"}) {
      if (!task_config[key]) {
        throw std::runtime_error("Config missing required key: 'tasks[" +
                                 task_id + "]." + key + "'");
      }
    }

    // Parse and store task
    this->tasks.emplace_back(
        aic_task_interfaces::build<aic_task_interfaces::msg::Task>()
            .id(task_id)
            .cable_type(task_config["cable_type"].as<std::string>())
            .cable_name(task_config["cable_name"].as<std::string>())
            .plug_type(task_config["plug_type"].as<std::string>())
            .plug_name(task_config["plug_name"].as<std::string>())
            .port_type(task_config["port_type"].as<std::string>())
            .port_name(task_config["port_name"].as<std::string>())
            .target_module_name(
                task_config["target_module_name"].as<std::string>())
            .time_limit(task_config["time_limit"].as<std::size_t>()));
  }

  config = _config;
  state = TrialState::Uninitialized;
}

//==============================================================================
Engine::Engine(const rclcpp::NodeOptions& options)
    : node_(std::make_shared<rclcpp::Node>("aic_engine", options)),
      wrench_sub_(nullptr),
      joint_state_sub_(nullptr),
      insert_cable_action_client_(nullptr),
      spawn_entity_client_(nullptr),
      is_first_trial_(true),
      active_trial_(std::nullopt),
      engine_state_(EngineState::Uninitialized) {
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

  spin_thread_ = std::thread([node = node_]() {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  });
}

//==============================================================================
void Engine::start() {
  switch (engine_state_) {
    case EngineState::Uninitialized:
      if (this->initialize() != EngineState::Initialized) {
        RCLCPP_ERROR(node_->get_logger(), "Engine failed to initialize");
        return;
      }
      [[fallthrough]];
    case EngineState::Initialized:
      this->run();
      break;
    case EngineState::Running:
      RCLCPP_WARN(node_->get_logger(), "Engine is already running");
      break;
    case EngineState::Error:
      RCLCPP_ERROR(node_->get_logger(),
                   "Engine is in error state. Cannot start.");
      break;
    case EngineState::Completed:
      RCLCPP_INFO(node_->get_logger(), "Engine has already completed.");
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown engine state. Cannot start.");
      break;
  }
}

//==============================================================================
EngineState Engine::initialize() {
  RCLCPP_INFO(node_->get_logger(), "Initializing AIC Engine...");

  // Initialize the trials.
  const std::filesystem::path config_file_path =
      node_->get_parameter("config_file_path").as_string();

  // Try to load config file as YAML
  try {
    config_ = YAML::LoadFile(config_file_path);
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load config file '%s': %s",
                 config_file_path.c_str(), e.what());
    engine_state_ = EngineState::Error;
    return engine_state_;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load config file '%s': %s",
                 config_file_path.c_str(), e.what());
    engine_state_ = EngineState::Error;
    return engine_state_;
  }

  // Parse trials from config
  if (!config_["trials"]) {
    RCLCPP_ERROR(node_->get_logger(), "Config missing required key: 'trials'");
    engine_state_ = EngineState::Error;
    return engine_state_;
  }

  const auto& trials_config = config_["trials"];
  for (auto it = trials_config.begin(); it != trials_config.end(); ++it) {
    const std::string trial_id = it->first.as<std::string>();
    const YAML::Node trial_config = it->second;

    try {
      Trial trial(trial_id, std::move(trial_config));
      trials_.emplace_back(trial_id, std::move(trial));
      RCLCPP_INFO(node_->get_logger(), "Successfully parsed trial '%s'",
                  trial_id.c_str());
    } catch (const std::runtime_error& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to parse trial '%s': %s",
                   trial_id.c_str(), e.what());
      engine_state_ = EngineState::Error;
      return engine_state_;
    }
  }

  if (trials_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No trials found in config");
    engine_state_ = EngineState::Error;
    return engine_state_;
  }

  RCLCPP_INFO(node_->get_logger(), "Successfully parsed %zu trial(s)",
              trials_.size());

  // Create ROS endpoints.
  const rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  wrench_sub_ = node_->create_subscription<WrenchStampedMsg>(
      "/axia80_m20/wrench", reliable_qos,
      [](WrenchStampedMsg::ConstSharedPtr msg) {
        (void)msg;
        // TODO(Yadunund): Pass to scoring.
      });
  joint_state_sub_ = node_->create_subscription<JointStateMsg>(
      "/joint_states", reliable_qos, [](JointStateMsg::ConstSharedPtr msg) {
        (void)msg;
        // TODO(Yadunund): Pass to scoring.
      });

  // Create subscriptions that ignore local publications as aic_engine will
  // also publish these messages to home the robot.
  rclcpp::SubscriptionOptions sub_options_ignore_local;
  sub_options_ignore_local.ignore_local_publications = true;
  joint_motion_update_sub_ = node_->create_subscription<JointMotionUpdateMsg>(
      "/aic_controller/joint_motion_update", reliable_qos,
      [this](JointMotionUpdateMsg::ConstSharedPtr msg) {
        last_joint_motion_update_msg_ = msg;
      },
      sub_options_ignore_local);
  motion_update_sub_ = node_->create_subscription<MotionUpdateMsg>(
      "/aic_controller/motion_update", reliable_qos,
      [this](MotionUpdateMsg::ConstSharedPtr msg) {
        last_motion_update_msg_ = msg;
      },
      sub_options_ignore_local);

  insert_cable_action_client_ =
      rclcpp_action::create_client<InsertCableAction>(node_, "/insert_cable");
  spawn_entity_client_ =
      node_->create_client<SpawnEntitySrv>("/gz_server/spawn_entity");
  delete_entity_client_ =
      node_->create_client<DeleteEntitySrv>("/gz_server/delete_entity");
  model_get_state_client_ = node_->create_client<lifecycle_msgs::srv::GetState>(
      model_get_state_service_name_);
  model_change_state_client_ =
      node_->create_client<lifecycle_msgs::srv::ChangeState>(
          model_change_state_service_name_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  engine_state_ = EngineState::Initialized;
  RCLCPP_INFO(node_->get_logger(), "AIC Engine initialized successfully.");

  return engine_state_;
}

//==============================================================================
EngineState Engine::run() {
  RCLCPP_INFO(node_->get_logger(), "Running AIC Engine...");

  engine_state_ = EngineState::Running;

  for (const auto& trial_entry : trials_) {
    const std::string& trial_id = trial_entry.first;
    const Trial& trial = trial_entry.second;
    RCLCPP_INFO(node_->get_logger(), "======================================");
    RCLCPP_INFO(node_->get_logger(), "Handling trial '%s'...",
                trial_id.c_str());
    TrialState trial_result = this->handle_trial(trial);
    if (trial_result == TrialState::TaskCompleted) {
      RCLCPP_INFO(node_->get_logger(), "Trial '%s' completed successfully.",
                  trial_id.c_str());
    } else {
      RCLCPP_ERROR(node_->get_logger(),
                   "Trial '%s' failed or was not completed.", trial_id.c_str());
      engine_state_ = EngineState::Error;
      // TODO(Yadunund): Clean up and write scoring data.
      return engine_state_;
    }
  }

  return engine_state_;
}

//==============================================================================
TrialState Engine::handle_trial(const Trial& trial) {
  RCLCPP_INFO(node_->get_logger(), "Starting trial '%s'...", trial.id.c_str());

  active_trial_ = trial;

  TrialState current_state = TrialState::Uninitialized;

  if (!this->check_model()) {
    RCLCPP_ERROR(node_->get_logger(), "Participant model is not ready.");
    reset_after_trial();
    return current_state;
  }
  current_state = TrialState::ModelReady;

  if (!this->check_endpoints()) {
    RCLCPP_ERROR(node_->get_logger(), "Required endpoints are not available.");
    reset_after_trial();
    return current_state;
  }
  current_state = TrialState::EndpointsReady;

  if (!this->ready_simulator()) {
    RCLCPP_ERROR(node_->get_logger(), "Simulator is not ready.");
    reset_after_trial();
    return current_state;
  }
  current_state = TrialState::SimulatorReady;

  if (!this->ready_scoring()) {
    RCLCPP_ERROR(node_->get_logger(), "Scoring system is not ready.");
    reset_after_trial();
    return current_state;
  }
  current_state = TrialState::ScoringReady;

  if (!this->start_task()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to start task.");
    reset_after_trial();
    return current_state;
  }
  current_state = TrialState::TaskStarted;

  if (!this->task_completed_successfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Task was not completed successfully.");
    reset_after_trial();
    return current_state;
  }
  current_state = TrialState::TaskCompleted;

  reset_after_trial();
  return current_state;
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

  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
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

  if (!model_change_state_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "ChangeState service not available for node '%s' after waiting",
        model_node_name_.c_str());
    return false;
  }

  // Create and send the request to transition to 'configured' state
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id =
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;

  auto future = model_change_state_client_->async_send_request(request);

  const int model_configure_timeout_seconds =
      node_->get_parameter("model_configure_timeout_seconds").as_int();
  if (future.wait_for(std::chrono::seconds(model_configure_timeout_seconds)) !=
      std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ChangeState service call timed out for node '%s'",
                 model_node_name_.c_str());
    return false;
  }

  auto response = future.get();

  if (!response->success) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to transition lifecycle node '%s' to 'configured' state",
        model_node_name_.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
              "Lifecycle node '%s' successfully transitioned to 'configured' "
              "state. Checking expectations...",
              model_node_name_.c_str());

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
  std::this_thread::sleep_for(std::chrono::seconds(1));

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

  // Check for lifecycle node by looking for its get_state service
  bool model_discovered = false;

  while (!model_discovered && !(this->node_->now() - start_time > timeout)) {
    RCLCPP_INFO(node_->get_logger(),
                "Checking if lifecycle node '%s' is available...",
                model_node_name_.c_str());
    const auto service_names_and_types = node_->get_service_names_and_types();
    auto it = service_names_and_types.find(model_get_state_service_name_);
    if (it != service_names_and_types.end()) {
      // Verify it's actually a lifecycle service by checking the type
      const auto& service_types = it->second;
      for (const auto& type : service_types) {
        if (type == "lifecycle_msgs/srv/GetState") {
          model_discovered = true;
          break;
        }
      }
    }
    if (!model_discovered) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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

  if (is_first_trial_ && !model_node_is_unconfigured()) {
    return false;
  }

  if (is_first_trial_ && !configure_model_node()) {
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

  // Check nodes
  std::set<std::string> unavailable = {this->adapter_node_name_};
  rclcpp::Time start_time = this->node_->now();
  const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(
      this->node_->get_parameter("endpoint_ready_timeout_seconds").as_int());
  const auto& node_graph = node_->get_node_graph_interface();

  while (!unavailable.empty() && !(this->node_->now() - start_time > timeout)) {
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
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!unavailable.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required nodes: %s",
                 string_set_to_csv(unavailable).c_str());
    return false;
  }

  // Check topics
  // TODO(Yadunund): Consider checking for messages received on topics.
  // unavailable is guaranteed to be empty here
  unavailable.insert({this->wrench_sub_->get_topic_name(),
                      this->joint_state_sub_->get_topic_name()});
  start_time = this->node_->now();
  while (!unavailable.empty() && !(this->node_->now() - start_time > timeout)) {
    if (this->wrench_sub_->get_publisher_count() > 0) {
      unavailable.erase(this->wrench_sub_->get_topic_name());
    }
    if (this->joint_state_sub_->get_publisher_count() > 0) {
      unavailable.erase(this->joint_state_sub_->get_topic_name());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!unavailable.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required topics: %s",
                 string_set_to_csv(unavailable).c_str());
    return false;
  }

  // Check services
  if (!spawn_entity_client_->wait_for_service(
          timeout.to_chrono<std::chrono::nanoseconds>())) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Spawn entity service not available after waiting");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "All required endpoints are available.");
  return true;
}

//==============================================================================
bool Engine::ready_simulator() {
  if (!this->active_trial_.has_value()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "No active trial set in engine. Report this bug.");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Readying simulator for trial '%s'...",
              this->active_trial_->id.c_str());

  // Spawn the task board.
  RCLCPP_INFO(node_->get_logger(), "Spawning task board.");
  const auto& task_board_config = active_trial_->config["scene"]["task_board"];
  if (this->spawn_entity("task_board", "/urdf/task_board.urdf.xacro",
                         task_board_config["pose"]["x"].as<double>(),
                         task_board_config["pose"]["y"].as<double>(),
                         task_board_config["pose"]["z"].as<double>(),
                         task_board_config["pose"]["roll"].as<double>(),
                         task_board_config["pose"]["pitch"].as<double>(),
                         task_board_config["pose"]["yaw"].as<double>())) {
    RCLCPP_INFO(node_->get_logger(), "Task board spawned successfully.");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to spawn task board.");
  }

  // Spawn the cable.
  RCLCPP_INFO(node_->get_logger(), "Spawning cable.");
  // Get the current gripper pose, and set the cable pose accordingly.
  std::string warning_msg;
  const std::string gripper_frame =
      node_->get_parameter("gripper_frame_name").as_string();
  if (!tf_buffer_->canTransform("world", gripper_frame, tf2::TimePointZero,
                                tf2::durationFromSec(1.0), &warning_msg)) {
    RCLCPP_WARN(node_->get_logger(), "TF Wait Failed: %s", warning_msg.c_str());
    return false;
  }
  geometry_msgs::msg::TransformStamped t =
      tf_buffer_->lookupTransform("world", gripper_frame, tf2::TimePointZero);
  const auto& cable_config = active_trial_->config["scene"]["cable"];
  if (this->spawn_entity(
          "cable", "/urdf/cable.sdf.xacro",
          t.transform.translation.x +
              cable_config["pose"]["gripper_offset"]["x"].as<double>(),
          t.transform.translation.y +
              cable_config["pose"]["gripper_offset"]["y"].as<double>(),
          t.transform.translation.z +
              cable_config["pose"]["gripper_offset"]["z"].as<double>(),
          cable_config["pose"]["roll"].as<double>(),
          cable_config["pose"]["pitch"].as<double>(),
          cable_config["pose"]["yaw"].as<double>())) {
    RCLCPP_INFO(node_->get_logger(), "Cable spawned successfully.");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to spawn cable.");
  }

  // TODO(Yadunund): Implement other simulator readiness checks.

  return true;
}

//==============================================================================
bool Engine::ready_scoring() {
  RCLCPP_INFO(node_->get_logger(), "Checking scoring system readiness...");

  // TODO(Yadunund): Implement actual scoring system readiness checks.
  std::this_thread::sleep_for(std::chrono::seconds(1));
  // For now, assume scoring system is ready.
  return true;
}

//==============================================================================
bool Engine::start_task() {
  RCLCPP_INFO(node_->get_logger(), "Starting task for active trial...");

  // TODO(Yadunund): Implement actual task start logic.
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // For now, assume task started successfully.
  return true;
}

//==============================================================================
bool Engine::task_completed_successfully() {
  RCLCPP_INFO(node_->get_logger(),
              "Checking if task was completed successfully...");

  // TODO(Yadunund): Implement actual task completion check.
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // For now, assume task was completed successfully.
  return true;
}

//==============================================================================
bool Engine::activate_model_node() {
  if (skip_model_ready_) {
    RCLCPP_INFO(node_->get_logger(),
                "Skipping model activation as per parameter.");
    return true;
  }

  RCLCPP_INFO(node_->get_logger(),
              "Activating model node '%s' to transition to 'active' state...",
              model_node_name_.c_str());

  auto change_state_request =
      std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  change_state_request->transition.id =
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

  const int model_activate_timeout_seconds =
      this->node_->get_parameter("model_activate_timeout_seconds").as_int();

  auto future =
      model_change_state_client_->async_send_request(change_state_request);
  if (future.wait_for(std::chrono::seconds(model_activate_timeout_seconds)) !=
      std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ChangeState service call timed out for activating node '%s'",
                 model_node_name_.c_str());
    return false;
  }

  auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to activate model node '%s'",
                 model_node_name_.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Successfully activated model node '%s'",
              model_node_name_.c_str());

  // TODO(Yadunund): Verify active requirements.
  return true;
}

//==============================================================================
bool Engine::deactivate_model_node() {
  if (skip_model_ready_) {
    RCLCPP_INFO(node_->get_logger(),
                "Skipping model deactivation as per parameter.");
    return true;
  }

  RCLCPP_INFO(
      node_->get_logger(),
      "Deactivating model node '%s' to transition to 'configured' state...",
      model_node_name_.c_str());

  auto change_state_request =
      std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  change_state_request->transition.id =
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;

  const int model_deactivate_timeout_seconds =
      this->node_->get_parameter("model_deactivate_timeout_seconds").as_int();

  auto future =
      model_change_state_client_->async_send_request(change_state_request);
  if (future.wait_for(std::chrono::seconds(model_deactivate_timeout_seconds)) !=
      std::future_status::ready) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "ChangeState service call timed out for deactivating node '%s'",
        model_node_name_.c_str());
    return false;
  }

  auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to deactivate model node '%s'",
                 model_node_name_.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Successfully deactivated model node '%s'",
              model_node_name_.c_str());
  return true;
}

//==============================================================================
void Engine::reset_after_trial() {
  RCLCPP_INFO(node_->get_logger(), "Resetting after trial completion...");

  // Deactivate the model node to transition back to configured state
  deactivate_model_node();

  // Remove spawned entities from simulator
  if (active_trial_.has_value()) {
    for (const auto& entity_name : active_trial_->spawned_entities) {
      // Delete spawned entity
      auto request = std::make_shared<DeleteEntitySrv::Request>();
      request->entity = entity_name;

      auto future = delete_entity_client_->async_send_request(request);
      if (future.wait_for(std::chrono::seconds(10)) !=
          std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Delete entity service call timed out for entity '%s'",
                     request->entity.c_str());
      } else {
        auto response = future.get();
        if (response->result.result !=
            simulation_interfaces::msg::Result::RESULT_OK) {  // RESULT_OK = 1
          RCLCPP_ERROR(node_->get_logger(), "Failed to delete entity '%s': %s",
                       request->entity.c_str(),
                       response->result.error_message.c_str());
        } else {
          RCLCPP_INFO(node_->get_logger(), "Successfully deleted entity '%s'",
                      request->entity.c_str());
        }
      }
    }
  }
  is_first_trial_ = false;
  active_trial_ = std::nullopt;
  RCLCPP_INFO(node_->get_logger(), "Reset after trial completed.");
}

//==============================================================================
Engine::~Engine() {
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

//==============================================================================
bool Engine::spawn_entity(std::string entity_name, std::string filepath,
                          double x, double y, double z, double roll,
                          double pitch, double yaw) {
  if (!active_trial_.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "No active trial to get config from");
    return false;
  }

  // Get the xacro file path
  const std::string aic_description_share =
      ament_index_cpp::get_package_share_directory("aic_description");
  const std::string xacro_file = aic_description_share + filepath;

  // Build xacro command with parameters from config
  std::stringstream cmd;
  cmd << "xacro " << xacro_file;

  const auto& config = active_trial_->config["scene"][entity_name];

  // Append entity-specific parameters
  if (entity_name == "cable") {
    // Add attach cable parameter
    bool attach_cable_to_gripper = config["attach_cable_to_gripper"].as<bool>();
    cmd << " attach_cable_to_gripper:="
        << (attach_cable_to_gripper ? "true" : "false");

    // Add cable type parameter
    std::string cable_type = config["cable_type"].as<std::string>();
    cmd << " cable_type:=" << cable_type;
  } else if (entity_name == "task_board") {
    // Read task board limits from config
    const auto& config_root = active_trial_->config;
    double nic_rail_min = -0.048;  // Default values
    double nic_rail_max = 0.036;
    double sc_rail_min = -0.055;
    double sc_rail_max = 0.055;
    double mount_rail_min = -0.09625;
    double mount_rail_max = 0.09625;

    if (config_root["task_board_limits"]) {
      const auto& limits = config_root["task_board_limits"];
      if (limits["nic_rail"]) {
        nic_rail_min = limits["nic_rail"]["min_translation"].as<double>();
        nic_rail_max = limits["nic_rail"]["max_translation"].as<double>();
      }
      if (limits["sc_rail"]) {
        sc_rail_min = limits["sc_rail"]["min_translation"].as<double>();
        sc_rail_max = limits["sc_rail"]["max_translation"].as<double>();
      }
      if (limits["mount_rail"]) {
        mount_rail_min = limits["mount_rail"]["min_translation"].as<double>();
        mount_rail_max = limits["mount_rail"]["max_translation"].as<double>();
      }
    }

    // Add NIC rail parameters (nic_rail_0 through nic_rail_4)
    for (int i = 0; i < 5; ++i) {
      std::string rail_key = "nic_rail_" + std::to_string(i);
      std::string mount_prefix = "nic_card_mount_" + std::to_string(i);

      if (config[rail_key] && config[rail_key]["entity_present"] &&
          config[rail_key]["entity_present"].as<bool>()) {
        cmd << " " << mount_prefix << "_present:=true";

        if (config[rail_key]["entity_pose"]) {
          const auto& pose = config[rail_key]["entity_pose"];

          double translation = pose["translation"].as<double>();
          // Clamp translation to NIC rail limits
          translation = std::clamp(translation, nic_rail_min, nic_rail_max);
          cmd << " " << mount_prefix << "_translation:=" << translation;

          // Add orientation parameters
          double roll = pose["roll"].as<double>();
          double pitch = pose["pitch"].as<double>();
          double yaw = pose["yaw"].as<double>();
          cmd << " " << mount_prefix << "_roll:=" << roll;
          cmd << " " << mount_prefix << "_pitch:=" << pitch;
          cmd << " " << mount_prefix << "_yaw:=" << yaw;
        }
      } else {
        cmd << " " << mount_prefix << "_present:=false";
      }
    }

    // Add SC rail parameters (sc_rail_0 and sc_rail_1)
    for (int i = 0; i < 2; ++i) {
      std::string rail_key = "sc_rail_" + std::to_string(i);
      std::string port_prefix = "sc_port_" + std::to_string(i);

      if (config[rail_key] && config[rail_key]["entity_present"] &&
          config[rail_key]["entity_present"].as<bool>()) {
        cmd << " " << port_prefix << "_present:=true";

        if (config[rail_key]["entity_pose"]) {
          const auto& pose = config[rail_key]["entity_pose"];

          double translation = pose["translation"].as<double>();
          // Clamp translation to SC rail limits
          translation = std::clamp(translation, sc_rail_min, sc_rail_max);
          cmd << " " << port_prefix << "_translation:=" << translation;

          // Add orientation parameters
          double roll = pose["roll"].as<double>();
          double pitch = pose["pitch"].as<double>();
          double yaw = pose["yaw"].as<double>();
          cmd << " " << port_prefix << "_roll:=" << roll;
          cmd << " " << port_prefix << "_pitch:=" << pitch;
          cmd << " " << port_prefix << "_yaw:=" << yaw;
        }
      } else {
        cmd << " " << port_prefix << "_present:=false";
      }
    }

    // Add rail parameters (type-specific rails: lc_mount_rail_0/1,
    // sfp_mount_rail_0/1, sc_mount_rail_0/1)
    std::vector<std::string> rail_keys = {
        "lc_mount_rail_0", "sfp_mount_rail_0", "sc_mount_rail_0",
        "lc_mount_rail_1", "sfp_mount_rail_1", "sc_mount_rail_1"};

    for (const auto& rail_key : rail_keys) {
      if (config[rail_key] && config[rail_key]["entity_present"] &&
          config[rail_key]["entity_present"].as<bool>()) {
        cmd << " " << rail_key << "_present:=true";

        if (config[rail_key]["entity_pose"]) {
          const auto& pose = config[rail_key]["entity_pose"];

          double translation = pose["translation"].as<double>();
          // Clamp translation to mount rail limits
          translation = std::clamp(translation, mount_rail_min, mount_rail_max);
          cmd << " " << rail_key << "_translation:=" << translation;

          // Add orientation parameters
          double roll = pose["roll"].as<double>();
          double pitch = pose["pitch"].as<double>();
          double yaw = pose["yaw"].as<double>();
          cmd << " " << rail_key << "_roll:=" << roll;
          cmd << " " << rail_key << "_pitch:=" << pitch;
          cmd << " " << rail_key << "_yaw:=" << yaw;
        }
      } else {
        cmd << " " << rail_key << "_present:=false";
      }
    }

    // Add ground_truth parameter
    cmd << " ground_truth:=" << (ground_truth_ ? "true" : "false");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Unknown entity name: %s",
                 entity_name.c_str());
    return false;
  }

  FILE* pipe = popen(cmd.str().c_str(), "r");
  if (!pipe) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to execute xacro command");
    return false;
  }

  std::stringstream urdf_stream;
  char buffer[128];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    urdf_stream << buffer;
  }
  int result = pclose(pipe);
  if (result != 0) {
    RCLCPP_ERROR(node_->get_logger(), "xacro command failed with code %d",
                 result);
    return false;
  }

  std::string urdf_string = urdf_stream.str();
  if (urdf_string.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Generated URDF is empty");
    return false;
  }

  // Convert roll, pitch, yaw to quaternion
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  double qw = cr * cp * cy + sr * sp * sy;
  double qx = sr * cp * cy - cr * sp * sy;
  double qy = cr * sp * cy + sr * cp * sy;
  double qz = cr * cp * sy - sr * sp * cy;

  // Create spawn request
  auto request = std::make_shared<SpawnEntitySrv::Request>();
  request->name = entity_name;
  request->allow_renaming = true;
  request->uri = "";
  request->resource_string = urdf_string;
  request->entity_namespace = "";
  request->initial_pose.header.frame_id = "world";
  request->initial_pose.pose.position.x = x;
  request->initial_pose.pose.position.y = y;
  request->initial_pose.pose.position.z = z;
  request->initial_pose.pose.orientation.x = qx;
  request->initial_pose.pose.orientation.y = qy;
  request->initial_pose.pose.orientation.z = qz;
  request->initial_pose.pose.orientation.w = qw;

  // Call service synchronously with timeout
  auto future = spawn_entity_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Spawn entity service call timed out");
    return false;
  }

  auto response = future.get();

  if (response->result.result !=
      simulation_interfaces::msg::Result::RESULT_OK) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to spawn cable: %s",
                 response->result.error_message.c_str());
    return false;
  }

  if (active_trial_.has_value()) {
    active_trial_->spawned_entities.emplace_back(response->entity_name);
  }

  RCLCPP_INFO(node_->get_logger(), "Successfully spawned %s as '%s'",
              entity_name.c_str(), response->entity_name.c_str());
  return true;
}

}  // namespace aic
