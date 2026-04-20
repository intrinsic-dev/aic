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
#include <mutex>
#include <sstream>
#include <unordered_set>

#include "aic_task_interfaces/msg/task.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/subscription_options.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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

  // Validate SC rails (sc_rail_0 and sc_rail_1), each with up to MAX_SC_PORTS_PER_RAIL ports
  constexpr int MAX_SC_PORTS_PER_RAIL = 3;
  for (int rail_idx = 0; rail_idx < 2; ++rail_idx) {
    std::string rail_key = "sc_rail_" + std::to_string(rail_idx);
    if (!task_board[rail_key]) {
      throw std::runtime_error(
          "Config missing required key: 'scene.task_board." + rail_key + "'");
    }
    const auto& rail = task_board[rail_key];
    int port_start = rail_idx * MAX_SC_PORTS_PER_RAIL;
    for (int port_idx = port_start; port_idx < port_start + MAX_SC_PORTS_PER_RAIL; ++port_idx) {
      std::string port_key = "sc_port_" + std::to_string(port_idx);
      if (!rail[port_key]) {
        throw std::runtime_error(
            "Config missing required key: 'scene.task_board." + rail_key + "." + port_key + "'");
      }
      if (!rail[port_key]["entity_present"]) {
        throw std::runtime_error(
            "Config missing required key: 'scene.task_board." + rail_key + "." + port_key +
            ".entity_present'");
      }
      if (rail[port_key]["entity_present"].as<bool>()) {
        if (!rail[port_key]["entity_name"]) {
          throw std::runtime_error(
              "Config missing required key: 'scene.task_board." + rail_key + "." + port_key +
              ".entity_name'");
        }
        if (!rail[port_key]["entity_pose"]) {
          throw std::runtime_error(
              "Config missing required key: 'scene.task_board." + rail_key + "." + port_key +
              ".entity_pose'");
        }
        const auto& entity_pose = rail[port_key]["entity_pose"];
        for (const auto& key : {"translation", "roll", "pitch", "yaw"}) {
          if (!entity_pose[key]) {
            throw std::runtime_error(
                "Config missing required key: 'scene.task_board." + rail_key + "." + port_key +
                ".entity_pose." + key + "'");
          }
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

  // Validate scene.cables
  if (!scene["cables"]) {
    throw std::runtime_error("Config missing required key: 'scene.cables'");
  }
  const auto& cables = scene["cables"];
  for (const auto& cable_it : cables) {
    const std::string cable_id = cable_it.first.as<std::string>();
    const YAML::Node cable = cable_it.second;
    if (!cable["pose"]) {
      throw std::runtime_error("Config missing required key: 'scene.cables[" +
                               cable_id + "].pose'");
    }
    const auto& cable_pose = cable["pose"];
    for (const auto& key : {"gripper_offset", "roll", "pitch", "yaw"}) {
      if (!cable_pose[key]) {
        throw std::runtime_error("Config missing required key: 'scene.cables[" +
                                 cable_id + "].pose." + key + "'");
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
      throw std::runtime_error("Config missing required key: 'scene.cables[" +
                               cable_id + "].attach_cable_to_gripper'");
    }
    if (!cable["cable_type"]) {
      throw std::runtime_error("Config missing required key: 'scene.cables[" +
                               cable_id + "].cable_type'");
    }
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
TaskAttempt::TaskAttempt(const std::string& _id)
    : id(std::move(_id)),
      time_started(std::nullopt),
      time_completed(std::nullopt),
      success(false),
      state(TaskState::Uninitialized) {
  //
}

//==============================================================================
YAML::Node Score::serialize() const {
  const double total_score = this->calculate_total_score();
  YAML::Node score;
  score["total"] = total_score;
  for (const auto& [trial_name, trial_score] : this->breakdown) {
    score[trial_name]["tier_1"] = trial_score.tier_1.to_yaml();
    score[trial_name]["tier_2"] = trial_score.tier_2.to_yaml();
    score[trial_name]["tier_3"] = trial_score.tier_3.to_yaml();
  }
  return score;
}

//==============================================================================
double Score::calculate_total_score() const {
  // TODO(luca) check calculation
  double score = 0;
  for (const auto& [trial_name, trial_score] : this->breakdown) {
    score += trial_score.tier_1.total_score();
    score += trial_score.tier_2.total_score();
    score += trial_score.tier_3.total_score();
  }
  return score;
}

//==============================================================================
Engine::Engine(const rclcpp::NodeOptions& options)
    : node_(std::make_shared<rclcpp::Node>("aic_engine", options)),
      insert_cable_action_client_(nullptr),
      spawn_entity_client_(nullptr),
      is_first_trial_(true),
      engine_state_(EngineState::Uninitialized),
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
  skip_ready_simulator_ =
      node_->declare_parameter("skip_ready_simulator", false);
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

  spin_thread_ = std::thread([node = node_]() {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  });
}

//==============================================================================
EngineState Engine::start() {
  switch (engine_state_) {
    case EngineState::Uninitialized:
      if (this->initialize() != EngineState::Initialized) {
        RCLCPP_ERROR(node_->get_logger(), "Engine failed to initialize");
        return engine_state_;
      }
      [[fallthrough]];
    case EngineState::Initialized:
      return this->run();
    case EngineState::Running:
      RCLCPP_WARN(node_->get_logger(), "Engine is already running");
      return engine_state_;
    case EngineState::Error:
      RCLCPP_ERROR(node_->get_logger(),
                   "Engine is in error state. Cannot start.");
      return engine_state_;
    case EngineState::Completed:
      RCLCPP_INFO(node_->get_logger(), "Engine has already completed.");
      return engine_state_;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown engine state. Cannot start.");
      return engine_state_;
  }
}

//==============================================================================
EngineState Engine::initialize() {
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;36m╔════════════════════════════════════════╗\033[0m");
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;36m║   Initializing AIC Engine...           ║\033[0m");
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;36m╚════════════════════════════════════════╝\033[0m");

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

  if (!config_["scoring"]) {
    RCLCPP_ERROR(node_->get_logger(), "Config missing required key: 'scoring'");
    engine_state_ = EngineState::Error;
    return engine_state_;
  }

  if (!config_["robot"]) {
    RCLCPP_ERROR(node_->get_logger(), "Config missing required key: 'robot'");
    engine_state_ = EngineState::Error;
    return engine_state_;
  }
  const auto& robot_config = config_["robot"];
  if (!robot_config["home_joint_positions"]) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Config missing required key: 'robot.home_joint_positions'");
    engine_state_ = EngineState::Error;
    return engine_state_;
  }

  // Make sure a valid clock is received, it takes time to initialize
  // the subscriber and following timeout calls might fail otherwise
  RCLCPP_INFO(node_->get_logger(), "Waiting for clock");
  if (!node_->get_clock()->wait_until_started(rclcpp::Duration(10, 0))) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to find a valid clock");
    return EngineState::Error;
  }
  RCLCPP_INFO(node_->get_logger(), "Clock found successfully.");

  // Create ROS endpoints.
  const rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

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
  switch_controller_client_ =
      node_->create_client<controller_manager_msgs::srv::SwitchController>(
          "/controller_manager/switch_controller");
  reset_joints_client_ =
      node_->create_client<ResetJointsSrv>("/scoring/reset_joints");
  tare_ft_client_ = node_->create_client<TriggerSrv>(
      "/aic_controller/tare_force_torque_sensor");
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Pre-build home messages directly from config
  const auto joint_config = robot_config["home_joint_positions"];
  if (joint_config.size() != 6) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Home joint positions should account for exactly 6 joints.");
    return EngineState::Error;
  }

  // Build JointMotionUpdateMsg and ResetJointsSrv::Request for homing
  auto home_point = JointTrajectoryPoint();
  home_point.positions = {};
  home_reset_joints_request_ = std::make_shared<ResetJointsSrv::Request>();

  for (auto it = joint_config.begin(); it != joint_config.end(); ++it) {
    const std::string joint_name = it->first.as<std::string>();
    const double initial_pos = it->second.as<double>();

    RCLCPP_INFO(node_->get_logger(),
                "Retrieved home joint position from config: [%s]: %f",
                joint_name.c_str(), initial_pos);

    home_point.positions.emplace_back(initial_pos);
    home_reset_joints_request_->joint_names.emplace_back(joint_name);
    home_reset_joints_request_->initial_positions.emplace_back(initial_pos);
  }

  home_joint_msg_.target_state = home_point;
  home_joint_msg_.target_stiffness = {100.0, 100.0, 100.0, 50.0, 50.0, 50.0};
  home_joint_msg_.target_damping = {40.0, 40.0, 40.0, 15.0, 15.0, 15.0};
  home_joint_msg_.trajectory_generation_mode.mode =
      TrajectoryGenerationMode::MODE_POSITION;

  RCLCPP_INFO(node_->get_logger(), "Pre-built home messages for robot homing.");

  scoring_tier2_ = std::make_unique<aic_scoring::ScoringTier2>(node_.get());
  if (!scoring_tier2_->Initialize(config_["scoring"])) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize scoring system");
    return EngineState::Error;
  }
  scoring_tier2_->SetGripperFrame(
      node_->get_parameter("gripper_frame_name").as_string());

  // Create output directory for bag files.
  std::error_code ec;
  std::filesystem::create_directories(scoring_output_dir_, ec);
  if (ec) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to create bag output directory '%s': %s",
                 scoring_output_dir_.c_str(), ec.message().c_str());
    return EngineState::Error;
  }
  RCLCPP_INFO(node_->get_logger(), "Bag output directory: %s",
              scoring_output_dir_.c_str());

  engine_state_ = EngineState::Initialized;
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;32m✓ AIC Engine initialized successfully!\033[0m");

  return engine_state_;
}

//==============================================================================
EngineState Engine::run() {
  RCLCPP_INFO(node_->get_logger(), " ");
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;35m╔════════════════════════════════════════╗\033[0m");
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;35m║      Starting AIC Engine Run           ║\033[0m");
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;35m║   Total Trials: %-3zu                    ║\033[0m",
              trials_.size());
  RCLCPP_INFO(node_->get_logger(),
              "\033[1;35m╚════════════════════════════════════════╝\033[0m");
  RCLCPP_INFO(node_->get_logger(), " ");

  engine_state_ = EngineState::Running;
  Score score;

  size_t trial_num = 1;
  for (auto& trial_entry : trials_) {
    if (!rclcpp::ok()) {
      engine_state_ = EngineState::Error;
      break;
    }
    const std::string& trial_id = trial_entry.first;
    Trial& trial = trial_entry.second;
    RCLCPP_INFO(node_->get_logger(), " ");
    RCLCPP_INFO(node_->get_logger(),
                "\033[1;33m━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\033[0m");
    RCLCPP_INFO(node_->get_logger(), "\033[1;33m  Trial %zu/%zu: %s\033[0m",
                trial_num++, trials_.size(), trial_id.c_str());
    RCLCPP_INFO(node_->get_logger(),
                "\033[1;33m━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\033[0m");
    TrialScore trial_score = this->handle_trial(trial);
    score.breakdown[trial_id] = trial_score;
    if (engine_state_ == EngineState::Error) {
      break;
    }

    if (trial.state == TrialState::AllTasksCompleted) {
      RCLCPP_INFO(
          node_->get_logger(),
          "\033[1;32m✓ Trial '%s' completed successfully! Score: %f\033[0m",
          trial_id.c_str(), trial_score.total_score());
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "\033[1;33m⚠ Trial '%s' failed or was not completed. Score: "
                  "%f. Continuing to next trial...\033[0m",
                  trial_id.c_str(), trial_score.total_score());
    }
  }

  // TODO(luca) refactor cleanup into single function
  this->cleanup_model_node();
  this->shutdown_model_node();
  this->validate_model_shutdown();
  this->score_run(score);

  // Count successful and failed trials
  size_t successful_trials = 0;
  size_t failed_trials = 0;
  for (const auto& trial_entry : trials_) {
    if (trial_entry.second.state == TrialState::AllTasksCompleted) {
      successful_trials++;
    } else {
      failed_trials++;
    }
  }

  RCLCPP_INFO(node_->get_logger(), " ");
  if (engine_state_ == EngineState::Running) {
    engine_state_ = EngineState::Completed;
    RCLCPP_INFO(node_->get_logger(),
                "\033[1;32m╔════════════════════════════════════════╗\033[0m");
    RCLCPP_INFO(node_->get_logger(),
                "\033[1;32m║   ✓ All Trials Processed!              ║\033[0m");
    RCLCPP_INFO(
        node_->get_logger(),
        "\033[1;32m║   Successful: %zu  Failed: %zu             ║\033[0m",
        successful_trials, failed_trials);
    RCLCPP_INFO(node_->get_logger(),
                "\033[1;32m║   Total Score: %.3f                     ║\033[0m",
                score.calculate_total_score());
    RCLCPP_INFO(node_->get_logger(),
                "\033[1;32m╚════════════════════════════════════════╝\033[0m");
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "\033[1;31m╔════════════════════════════════════════╗\033[0m");
    RCLCPP_ERROR(node_->get_logger(),
                 "\033[1;31m║   ✗ Engine Stopped with Errors         ║\033[0m");
    RCLCPP_ERROR(node_->get_logger(),
                 "\033[1;31m╚════════════════════════════════════════╝\033[0m");
  }
  RCLCPP_INFO(node_->get_logger(), " ");

  // Print full scoring breakdown
  YAML::Node scoring_yaml = score.serialize();
  std::stringstream ss;
  ss << scoring_yaml;
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

  return engine_state_;
}

//==============================================================================
TrialScore Engine::handle_trial(Trial& trial) {
  RCLCPP_INFO(node_->get_logger(), "\033[1;36m→ Starting trial '%s'...\033[0m",
              trial.id.c_str());
  TrialScore score;

  constexpr int MAX_RETRIES = 5;

  if (trial.state == TrialState::Uninitialized) {
    if (this->check_model()) {
      trial.state = TrialState::ModelReady;
    }
  } else {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Attempted to start trial while not Uninitialized. Report this bug.");
    reset_after_trial(trial);
    engine_state_ = EngineState::Error;
    return score;
  }

  if (trial.state == TrialState::ModelReady) {
    RCLCPP_INFO(node_->get_logger(),
                "\033[1;32m  ✓ Model Ready for trial '%s'\033[0m",
                trial.id.c_str());
    score.tier_1_success();
    bool success = false;
    for (int attempt = 1; attempt <= MAX_RETRIES && !success; ++attempt) {
      if (attempt > 1) {
        RCLCPP_WARN(node_->get_logger(),
                    "\033[1;33m  ⟳ Retrying check_endpoints (attempt %d/%d) "
                    "for trial '%s'...\033[0m",
                    attempt, MAX_RETRIES, trial.id.c_str());
      }
      if (this->check_endpoints()) {
        trial.state = TrialState::EndpointsReady;
        success = true;
      }
    }
    if (!success) {
      RCLCPP_ERROR(node_->get_logger(),
                   "\033[1;31m  ✗ EVALUATION ERROR: Endpoints check failed "
                   "after %d attempts for trial '%s'. "
                   "This is an infrastructure issue. Is eval environment "
                   "started?\033[0m",
                   MAX_RETRIES, trial.id.c_str());
      reset_after_trial(trial);
      engine_state_ = EngineState::Error;
      return score;
    }
  } else {
    RCLCPP_ERROR(
        node_->get_logger(),
        "\033[1;31m  ✗ Participant model is not ready for trial '%s'\033[0m",
        trial.id.c_str());
    reset_after_trial(trial);
    engine_state_ = EngineState::Error;
    return score;
  }

  if (trial.state == TrialState::EndpointsReady) {
    RCLCPP_INFO(node_->get_logger(),
                "\033[1;32m  ✓ Endpoints Ready for trial '%s'\033[0m",
                trial.id.c_str());
    bool success = false;
    for (int attempt = 1; attempt <= MAX_RETRIES && !success; ++attempt) {
      if (attempt > 1) {
        RCLCPP_WARN(node_->get_logger(),
                    "\033[1;33m  ⟳ Retrying ready_simulator (attempt %d/%d) "
                    "for trial '%s'...\033[0m",
                    attempt, MAX_RETRIES, trial.id.c_str());
        // Reset simulator before retrying (don't home robot during retry)
        reset_simulator(trial, false);
      }
      if (this->ready_simulator(trial)) {
        trial.state = TrialState::SimulatorReady;
        success = true;
      }
    }
    if (!success) {
      RCLCPP_ERROR(node_->get_logger(),
                   "\033[1;31m  ✗ EVALUATION ERROR: Simulator setup failed "
                   "after %d attempts for trial '%s'. "
                   "This is an infrastructure issue. Is eval environment "
                   "started?\033[0m",
                   MAX_RETRIES, trial.id.c_str());
      reset_after_trial(trial);
      engine_state_ = EngineState::Error;
      return score;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "\033[1;31m  ✗ Required endpoints are not available for trial "
                 "'%s'\033[0m",
                 trial.id.c_str());
    reset_after_trial(trial);
    engine_state_ = EngineState::Error;
    return score;
  }

  if (trial.state == TrialState::SimulatorReady) {
    RCLCPP_INFO(node_->get_logger(),
                "\033[1;32m  ✓ Simulator Ready for trial '%s'\033[0m",
                trial.id.c_str());
    bool success = false;
    for (int attempt = 1; attempt <= MAX_RETRIES && !success; ++attempt) {
      if (attempt > 1) {
        RCLCPP_WARN(node_->get_logger(),
                    "\033[1;33m  ⟳ Retrying ready_scoring (attempt %d/%d) for "
                    "trial '%s'...\033[0m",
                    attempt, MAX_RETRIES, trial.id.c_str());
      }
      if (this->ready_scoring(trial)) {
        trial.state = TrialState::ScoringReady;
        success = true;
      }
    }
    if (!success) {
      RCLCPP_ERROR(node_->get_logger(),
                   "\033[1;31m  ✗ EVALUATION ERROR: Scoring setup failed after "
                   "%d attempts for trial '%s'. "
                   "This is an infrastructure issue. Is eval environment "
                   "started?\033[0m",
                   MAX_RETRIES, trial.id.c_str());
      reset_after_trial(trial);
      engine_state_ = EngineState::Error;
      return score;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "\033[1;31m  ✗ Simulator is not ready for trial '%s'\033[0m",
                 trial.id.c_str());
    reset_after_trial(trial);
    engine_state_ = EngineState::Error;
    return score;
  }

  if (trial.state == TrialState::ScoringReady) {
    RCLCPP_INFO(node_->get_logger(),
                "\033[1;32m  ✓ Scoring Ready for trial '%s'\033[0m",
                trial.id.c_str());
    this->tasks_started(trial);
  } else {
    RCLCPP_ERROR(
        node_->get_logger(),
        "\033[1;31m  ✗ Scoring system is not ready for trial '%s'\033[0m",
        trial.id.c_str());
    reset_after_trial(trial);
    engine_state_ = EngineState::Error;
    return score;
  }

  if (trial.state == TrialState::TasksExecuting) {
    RCLCPP_INFO(node_->get_logger(),
                "\033[1;36m  ⟳ Tasks Executing for trial '%s'...\033[0m",
                trial.id.c_str());
    if (this->tasks_completed_successfully(trial)) {
      trial.state = TrialState::AllTasksCompleted;
      RCLCPP_INFO(node_->get_logger(),
                  "\033[1;32m  ✓ All Tasks Completed for trial '%s'!\033[0m",
                  trial.id.c_str());
      score_trial(score);
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "\033[1;31m  ✗ Tasks cannot be started successfully for trial "
                 "'%s'\033[0m",
                 trial.id.c_str());
    reset_after_trial(trial);
    engine_state_ = EngineState::Error;
    return score;
  }

  if (trial.state != TrialState::AllTasksCompleted) {
    RCLCPP_ERROR(node_->get_logger(),
                 "\033[1;31m  ✗ Tasks were not completed successfully for "
                 "trial '%s'\033[0m",
                 trial.id.c_str());
    score_trial(score);
    reset_after_trial(trial);
    return score;
  }

  reset_after_trial(trial);
  return score;
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
bool Engine::ready_simulator(Trial& trial) {
  RCLCPP_INFO(node_->get_logger(), "Readying simulator for trial '%s'...",
              trial.id.c_str());

  if (skip_ready_simulator_) {
    RCLCPP_WARN(node_->get_logger(),
                "Skipping ready_simulator (skip_ready_simulator=true)");
    return true;
  }

  // Spawn the task board.
  RCLCPP_INFO(node_->get_logger(), "Spawning task board.");
  const auto& task_board_config = trial.config["scene"]["task_board"];
  if (this->spawn_entity(trial, "task_board", "/urdf/task_board.urdf.xacro",
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

  // Tare the force-torque sensor before attaching any cables to compensate for
  // the weight of the end-effector.
  const auto tare_req = std::make_shared<TriggerSrv::Request>();
  auto tare_ft_future = tare_ft_client_->async_send_request(tare_req);
  if (!wait_for_interruptible(tare_ft_future, std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "TareFt service call timed out requesting for taring!");
    return false;
  }
  auto tare_ft_response = tare_ft_future.get();
  if (!tare_ft_response->success) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to request for taring.");
    return false;
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
  const auto& cables_config = trial.config["scene"]["cables"];
  bool cable_attached = false;
  for (const auto& cable_it : cables_config) {
    const std::string cable_id = cable_it.first.as<std::string>();
    const YAML::Node cable_config = cable_it.second;
    bool attach_to_gripper = cable_config["attach_cable_to_gripper"].as<bool>();
    if (cable_attached && attach_to_gripper) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Attempting to attach multiple cables to the gripper. "
                   "Please check the config.");
      return false;
    } else if (attach_to_gripper) {
      cable_attached = true;
    }
    RCLCPP_INFO(node_->get_logger(), "Spawning cable '%s'...",
                cable_id.c_str());
    if (this->spawn_entity(
            trial, cable_id, "/urdf/cable.sdf.xacro",
            t.transform.translation.x +
                cable_config["pose"]["gripper_offset"]["x"].as<double>(),
            t.transform.translation.y +
                cable_config["pose"]["gripper_offset"]["y"].as<double>(),
            t.transform.translation.z +
                cable_config["pose"]["gripper_offset"]["z"].as<double>(),
            cable_config["pose"]["roll"].as<double>(),
            cable_config["pose"]["pitch"].as<double>(),
            cable_config["pose"]["yaw"].as<double>())) {
      RCLCPP_INFO(node_->get_logger(), "Cable %s spawned successfully.",
                  cable_id.c_str());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to spawn cable %s.",
                   cable_id.c_str());
      return false;
    }
  }

  // Wait for cable to be spawned. Sleep in sim time to ensure sim has advanced.
  node_->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.1));

  RCLCPP_INFO(node_->get_logger(), "Waiting for robot arm to stabilize.");
  // The end-effector dips when the cable is first attached.
  // Wait for joints to settle by checking velocities.
  std::condition_variable cv;
  std::mutex mtx;
  bool joints_settled = false;
  const rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  auto joint_states_sub =
      node_->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", reliable_qos,
          [this, &joints_settled, &cv,
           &mtx](const sensor_msgs::msg::JointState::SharedPtr msg) {
            if (msg->velocity.empty()) return;
            for (size_t i = 0; i < msg->velocity.size(); ++i) {
              if (std::fabs(msg->velocity[i]) > 1e-3) return;
            }
            {
              std::unique_lock<std::mutex> lock(mtx);
              joints_settled = true;
            }
            cv.notify_one();
          });

  std::unique_lock<std::mutex> lock(mtx);
  const auto start = node_->now();
  const auto timeout_duration = rclcpp::Duration::from_seconds(10);
  while (rclcpp::ok() && !joints_settled &&
         (node_->now() - start) < timeout_duration) {
    cv.wait_for(lock, std::chrono::milliseconds(100),
                [&joints_settled] { return joints_settled; });
  }

  // TODO(Yadunund): Implement other simulator readiness checks.

  return true;
}

//==============================================================================
bool Engine::ready_scoring(const Trial& trial) {
  RCLCPP_INFO(node_->get_logger(), "Checking scoring system readiness...");
  // Register the new connections for this trial.
  std::vector<aic_scoring::Connection> connections;
  for (const auto& task : trial.tasks) {
    aic_scoring::Connection connection;
    connection.cableName = task.cable_name;
    connection.taskBoardName = "task_board";
    connection.plugName = task.plug_name;
    connection.portName = task.port_name;
    connection.targetModuleName = task.target_module_name;
    connections.push_back(connection);
  }

  // Create unique bag filename with timestamp
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) %
            1000;

  std::ostringstream oss;
  oss << scoring_output_dir_ << "/bag_" << trial.id << "_"
      << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_"
      << std::setfill('0') << std::setw(3) << ms.count();
  const std::string bag_path = oss.str();

  unsigned int max_task_limit = 0;
  for (const auto& task : trial.tasks) {
    if (task.time_limit > max_task_limit) {
      max_task_limit = task.time_limit;
    }
  }
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
bool Engine::tasks_started(Trial& trial) {
  RCLCPP_INFO(node_->get_logger(), "Starting tasks for active trial...");

  if (trial.tasks.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "No task provided for this trial. Please check the config");
    return false;
  }
  if (!trial.attempts.empty()) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "List of attempts non-empty before starting tasks. Report this bug.");
    return false;
  }

  for (auto& task : trial.tasks) {
    // Initialize TaskState
    TaskAttempt task_attempt(task.id);
    trial.attempts.emplace_back(std::move(task_attempt));
    auto& current_attempt = trial.attempts.back();

    auto insert_cable_goal = InsertCableAction::Goal();
    insert_cable_goal.task = task;

    RCLCPP_INFO(this->node_->get_logger(),
                "Sending InsertCable goal for task [%s]", task.id.c_str());
    auto send_goal_future =
        insert_cable_action_client_->async_send_goal(insert_cable_goal);
    current_attempt.state = TaskState::TaskRequested;

    // Handle goal response
    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->node_->get_logger(),
                   "InsertCable goal for task [%s] was rejected.",
                   task.id.c_str());
      current_attempt.state = TaskState::TaskRejected;
      return false;
    }
    current_attempt.time_started = this->node_->now();
    current_attempt.state = TaskState::TaskStarted;
    // TODO(luca) Scoring assumes a single task per trial, revisit this
    // when this is not the case anymore
    this->scoring_tier2_->SetTaskStartTime(*current_attempt.time_started);

    // Update trial state
    trial.state = TrialState::TasksExecuting;
    RCLCPP_INFO(this->node_->get_logger(), "TrialState: TasksExecuting");

    // Handle goal result
    auto result_future =
        insert_cable_action_client_->async_get_result(goal_handle);
    RCLCPP_INFO(this->node_->get_logger(), "Waiting for result...");

    // Cancel goal if time limit exceeded
    if (!wait_for_interruptible(result_future,
                                std::chrono::seconds(task.time_limit))) {
      RCLCPP_ERROR(this->node_->get_logger(),
                   "Task [%s] timed out after %ld seconds. Cancelling goal.",
                   task.id.c_str(), task.time_limit);
      insert_cable_action_client_->async_cancel_goal(goal_handle);
      current_attempt.state = TaskState::TimeLimitExceeded;
      return false;
    }

    auto result = result_future.get();
    if (!result.result->success) {
      RCLCPP_INFO(this->node_->get_logger(), "Task [%s] failed: %s",
                  task.id.c_str(), result.result->message.c_str());
      current_attempt.state = TaskState::TaskFailed;
      return false;
    }

    // Task succeeded, move off and send the next task goal
    RCLCPP_INFO(this->node_->get_logger(), "Task [%s] succeeded.",
                task.id.c_str());
    current_attempt.time_completed = this->node_->now();
    this->scoring_tier2_->SetTaskEndTime(*current_attempt.time_completed);
    current_attempt.state = TaskState::TaskCompleted;
  }

  RCLCPP_INFO(node_->get_logger(), "All tasks have been processed.");
  trial.tasks.clear();
  return true;
}

//==============================================================================
bool Engine::tasks_completed_successfully(const Trial& trial) {
  RCLCPP_INFO(node_->get_logger(),
              "Checking if all tasks were completed successfully...");

  // Check that there are no tasks left in queue
  if (!trial.tasks.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "There are still pending tasks in the active trial.");
    return false;
  }
  // Check that all tasks were completed successfully
  for (const auto& attempt : trial.attempts) {
    if (attempt.state != TaskState::TaskCompleted) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Task [%s] was not completed successfully. Last logged "
                   "TaskState was [%d].",
                   attempt.id.c_str(), static_cast<int>(attempt.state));
      return false;
    }
    if (!attempt.time_started.has_value() ||
        !attempt.time_completed.has_value()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Task [%s] is marked as completed but missing start or "
                   "completion time.Report this bug.",
                   attempt.id.c_str());
      return false;
    }
    if (attempt.time_completed <= attempt.time_started) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Task [%s] has invalid completion time. Report this bug.",
                   attempt.id.c_str());
      return false;
    }
  }
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

  if (!this->model_discovered_) {
    return true;
  }

  return this->transition_model_lifecycle_node(
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
}

//==============================================================================
bool Engine::shutdown_model_node() {
  if (skip_model_ready_) {
    RCLCPP_INFO(node_->get_logger(),
                "Skipping model shutdown as per parameter.");
    return true;
  }

  if (!this->model_discovered_) {
    return true;
  }

  return this->transition_model_lifecycle_node(
      lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}

//==============================================================================
bool Engine::validate_model_shutdown() const {
  if (skip_model_ready_) {
    RCLCPP_INFO(node_->get_logger(),
                "Skipping model shutdown validation as per parameter.");
    return true;
  }

  if (!this->model_discovered_) {
    return true;
  }

  auto node_graph = node_->get_node_graph_interface();
  const auto start = node_->now();
  const auto timeout_duration = rclcpp::Duration::from_seconds(2);

  while (rclcpp::ok() && (node_->now() - start) < timeout_duration) {
    const std::size_t pose_pubs =
        node_graph->count_publishers("/aic_controller/pose_commands");
    const std::size_t joint_pubs =
        node_graph->count_publishers("/aic_controller/joint_commands");
    if (pose_pubs == 0 && joint_pubs == 0) {
      return true;
    }
    node_->get_clock()->sleep_for(
        rclcpp::Duration(std::chrono::milliseconds(100)));
  }

  // Timed out — report all violations
  const std::size_t pose_pubs =
      node_graph->count_publishers("/aic_controller/pose_commands");
  const std::size_t joint_pubs =
      node_graph->count_publishers("/aic_controller/joint_commands");
  if (pose_pubs > 0) {
    RCLCPP_WARN(node_->get_logger(),
                "Detected %zu pose command publishers in shutdown state, this "
                "is not allowed and might affect scoring in the future",
                pose_pubs);
  }
  if (joint_pubs > 0) {
    RCLCPP_WARN(node_->get_logger(),
                "Detected %zu joint command publishers in shutdown state, "
                "this is not allowed and might affect scoring in the future",
                joint_pubs);
  }
  return false;
}

//==============================================================================
void Engine::reset_after_trial(const Trial& trial) {
  RCLCPP_INFO(node_->get_logger(), "Resetting after trial completion...");

  // Deactivate the model node to transition back to configured state
  if (this->model_discovered_) {
    this->deactivate_model_node();
  }

  is_first_trial_ = false;

  reset_simulator(trial);  // Homes robot by default

  RCLCPP_INFO(node_->get_logger(), "Reset after trial completed.");
}

//==============================================================================
bool Engine::home_robot() {
  RCLCPP_INFO(node_->get_logger(), "Homing robot to initial positions...");

  // Lambda to switch controllers
  auto switch_controllers =
      [this](const std::vector<std::string>& activate,
             const std::vector<std::string>& deactivate) -> bool {
    auto request = std::make_shared<SwitchControllerSrv::Request>();
    request->activate_controllers = activate;
    request->deactivate_controllers = deactivate;
    request->strictness = SwitchControllerSrv::Request::BEST_EFFORT;

    auto future = switch_controller_client_->async_send_request(request);
    if (!wait_for_interruptible(future, std::chrono::seconds(10))) {
      RCLCPP_ERROR(node_->get_logger(),
                   "SwitchController service call timed out");
      return false;
    }

    auto response = future.get();
    if (!response->ok) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to switch controllers.");
      return false;
    }

    return true;
  };

  // Deactivate aic_controller
  if (!switch_controllers({}, {"aic_controller"})) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to deactivate aic_controller.");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "aic_controller deactivated successfully.");

  // Request for joints reset to home positions using pre-built request.
  auto reset_joints_future =
      reset_joints_client_->async_send_request(home_reset_joints_request_);
  if (!wait_for_interruptible(reset_joints_future, std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ResetJoints service call timed out requesting for reset!");
    return false;
  }
  auto reset_joints_response = reset_joints_future.get();
  if (!reset_joints_response->success) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to request for joint reset.");
    return false;
  }

  // Activate aic_controller & resume simulation
  if (!switch_controllers({"aic_controller"}, {})) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to activate aic_controller.");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "aic_controller activated successfully.");

  RCLCPP_INFO(
      node_->get_logger(),
      "Successfully reset joints to home position, robot homed successfully.");
  return true;
}

//==============================================================================
void Engine::reset_simulator(const Trial& trial, bool home_robot) {
  if (skip_ready_simulator_) {
    RCLCPP_WARN(node_->get_logger(),
                "Skipping entity deletion (skip_ready_simulator=true)");
    return;
  }

  // Remove spawned entities from simulator
  for (const auto& entity_name : trial.spawned_entities) {
    // Delete spawned entity
    auto request = std::make_shared<DeleteEntitySrv::Request>();
    request->entity = entity_name;

    auto future = delete_entity_client_->async_send_request(request);
    if (!wait_for_interruptible(future, std::chrono::seconds(10))) {
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

  // Home robot after removing entities to prepare for next trial
  if (home_robot) {
    if (!this->home_robot()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Failed to home robot during simulator reset.");
    }
  }
}

//==============================================================================
Engine::~Engine() {
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

//==============================================================================
bool Engine::spawn_entity(Trial& trial, std::string entity_name,
                          std::string filepath, double x, double y, double z,
                          double roll, double pitch, double yaw) {
  // Get the xacro file path
  const std::string aic_description_share =
      ament_index_cpp::get_package_share_directory("aic_description");
  const std::string xacro_file = aic_description_share + filepath;

  // Build xacro command with parameters from config
  std::stringstream cmd;
  cmd << "xacro " << xacro_file;

  const auto& config = trial.config["scene"][entity_name];

  // Append entity-specific parameters
  if (entity_name.find("cable") != std::string::npos) {
    const auto& config = trial.config["scene"]["cables"][entity_name];
    // Add attach cable parameter
    bool attach_cable_to_gripper = config["attach_cable_to_gripper"].as<bool>();
    cmd << " attach_cable_to_gripper:="
        << (attach_cable_to_gripper ? "true" : "false");

    // Add cable type parameter
    std::string cable_type = config["cable_type"].as<std::string>();
    cmd << " cable_type:=" << cable_type;
  } else if (entity_name == "task_board") {
    const auto& config = trial.config["scene"][entity_name];
    // Read task board limits from config
    const auto& config_root = trial.config;
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

    // Add SC rail parameters (sc_rail_0 and sc_rail_1, up to MAX_SC_PORTS_PER_RAIL ports each)
    constexpr double SC_PORT_MIN_BUFFER = 0.030;
    constexpr int MAX_SC_PORTS_PER_RAIL = 3;
    for (int rail_idx = 0; rail_idx < 2; ++rail_idx) {
      std::string rail_key = "sc_rail_" + std::to_string(rail_idx);
      int port_start = rail_idx * MAX_SC_PORTS_PER_RAIL;

      // Collect present ports and clamp translations to rail limits
      std::vector<std::pair<int, double>> present_ports;
      for (int port_idx = port_start; port_idx < port_start + MAX_SC_PORTS_PER_RAIL; ++port_idx) {
        std::string port_key = "sc_port_" + std::to_string(port_idx);
        if (config[rail_key] && config[rail_key][port_key] &&
            config[rail_key][port_key]["entity_present"] &&
            config[rail_key][port_key]["entity_present"].as<bool>()) {
          double t = config[rail_key][port_key]["entity_pose"]["translation"].as<double>();
          t = std::clamp(t, sc_rail_min, sc_rail_max);
          present_ports.push_back({port_idx, t});
        }
      }

      // Sort by translation so buffer enforcement is applied in order
      std::sort(present_ports.begin(), present_ports.end(),
                [](const auto& a, const auto& b) { return a.second < b.second; });

      // Forward-pass: push ports forward to enforce minimum buffer
      bool adjusted_positions = false;
      for (size_t j = 1; j < present_ports.size(); ++j) {
        if (present_ports[j].second - present_ports[j - 1].second < SC_PORT_MIN_BUFFER) {
          present_ports[j].second = present_ports[j - 1].second + SC_PORT_MIN_BUFFER;
          adjusted_positions = true;
        }
      }

      // Backward-pass: if last port exceeds rail max, pull all back while maintaining gaps
      if (!present_ports.empty() && present_ports.back().second > sc_rail_max) {
        adjusted_positions = true;
        RCLCPP_WARN(node_->get_logger(),
                    "SC %s: port translations exceed rail max (%.3f m); port positions have been "
                    "adjusted to fit within rail limits.",
                    rail_key.c_str(), sc_rail_max);
        present_ports.back().second = sc_rail_max;
        for (int j = static_cast<int>(present_ports.size()) - 2; j >= 0; --j) {
          double required = present_ports[j + 1].second - SC_PORT_MIN_BUFFER;
          if (present_ports[j].second > required)
            present_ports[j].second = required;
        }
      }

      // Final validation: if first port fell below rail min, ports cannot all fit
      if (!present_ports.empty() && present_ports.front().second < sc_rail_min) {
        throw std::runtime_error(
            rail_key + ": present ports cannot all fit within rail limits [" +
            std::to_string(sc_rail_min) + ", " + std::to_string(sc_rail_max) +
            "] with minimum buffer " + std::to_string(SC_PORT_MIN_BUFFER) + " m.");
      }

      // Defensive check: assert no overlap remains after adjustment
      for (size_t j = 1; j < present_ports.size(); ++j) {
        if (present_ports[j].second - present_ports[j - 1].second < SC_PORT_MIN_BUFFER - 1e-9) {
          throw std::runtime_error(
              rail_key + ": buffer enforcement failed — ports still overlap after adjustment.");
        }
      }

      if (adjusted_positions) {
        RCLCPP_WARN(node_->get_logger(),
                    "SC %s: one or more port translations were adjusted to enforce %.3f m minimum "
                    "buffer and stay within rail limits [%.3f, %.3f] m.",
                    rail_key.c_str(), SC_PORT_MIN_BUFFER, sc_rail_min, sc_rail_max);
      }

      // Build port-index to adjusted translation map
      std::map<int, double> adjusted;
      for (const auto& [idx, t] : present_ports) adjusted[idx] = t;

      // Emit xacro params for all ports on this rail
      for (int port_idx = port_start; port_idx < port_start + MAX_SC_PORTS_PER_RAIL; ++port_idx) {
        std::string port_key = "sc_port_" + std::to_string(port_idx);
        std::string prefix = "sc_port_" + std::to_string(port_idx);
        if (adjusted.count(port_idx)) {
          cmd << " " << prefix << "_present:=true";
          cmd << " " << prefix << "_translation:=" << adjusted[port_idx];
          const auto& pose = config[rail_key][port_key]["entity_pose"];
          cmd << " " << prefix << "_roll:=" << pose["roll"].as<double>();
          cmd << " " << prefix << "_pitch:=" << pose["pitch"].as<double>();
          cmd << " " << prefix << "_yaw:=" << pose["yaw"].as<double>();
        } else {
          cmd << " " << prefix << "_present:=false";
        }
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

  // Add entity to spawned_entities list before making the service call
  // This ensures cleanup will attempt to delete it even if the service
  // reports failure but entity actually spawns in the background
  trial.spawned_entities.emplace_back(entity_name);

  // Call service synchronously with timeout
  auto future = spawn_entity_client_->async_send_request(request);
  if (!wait_for_interruptible(future, std::chrono::seconds(10))) {
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

  RCLCPP_INFO(node_->get_logger(), "Successfully spawned %s as '%s'",
              entity_name.c_str(), response->entity_name.c_str());
  return true;
}

//==============================================================================
void Engine::score_trial(TrialScore& score) {
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
void Engine::score_run(const Score& score) {
  YAML::Node node = score.serialize();

  const std::string yaml_output_file = scoring_output_dir_ + "/scoring.yaml";
  std::ofstream fout(yaml_output_file);
  fout << node;
}

}  // namespace aic
