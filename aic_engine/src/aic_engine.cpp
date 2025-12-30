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

namespace aic {

//==============================================================================
namespace {
// Static arrays for ROS graph entities to check for
// TODO(Yadunund): Uncomment and fill in required nodes when available.
static const std::vector<std::string> REQUIRED_NODES = {
    // "/aic_adapter_node",
    // "/aic_model_node",
};

}  // anonymous namespace

//==============================================================================
Trial::Trial(const std::string& _id, YAML::Node _config)
    : id(std::move(_id)), spawned_task_board_name(std::nullopt) {
  // Validate config structure
  if (!_config["scene"]) {
    throw std::runtime_error("Config missing required key: 'scene'");
  }
  if (!_config["tasks"]) {
    throw std::runtime_error("Config missing required key: 'tasks'");
  }
  if (!_config["scoring"]) {
    throw std::runtime_error("Config missing required key: 'scoring'");
  }

  // Validate scene.task_board
  const auto& scene = _config["scene"];
  if (!scene["task_board"]) {
    throw std::runtime_error("Config missing required key: 'scene.task_board'");
  }
  const auto& task_board = scene["task_board"];
  if (!task_board["pose"]) {
    throw std::runtime_error(
        "Config missing required key: 'scene.task_board.pose'");
  }
  const auto& pose = task_board["pose"];
  for (const auto& key : {"x", "y", "z", "roll", "pitch", "yaw"}) {
    if (!pose[key]) {
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
    auto task = aic_task_interfaces::build<aic_task_interfaces::msg::Task>()
                    .id(task_id)
                    .cable_type(task_config["cable_type"].as<std::string>())
                    .cable_name(task_config["cable_name"].as<std::string>())
                    .plug_type(task_config["plug_type"].as<std::string>())
                    .plug_name(task_config["plug_name"].as<std::string>())
                    .port_type(task_config["port_type"].as<std::string>())
                    .port_name(task_config["port_name"].as<std::string>())
                    .target_module_name(
                        task_config["target_module_name"].as<std::string>())
                    .time_limit(task_config["time_limit"].as<std::size_t>());

    this->tasks[task_id] = task;
  }

  // Validate scoring array
  const auto& scoring = _config["scoring"];
  if (!scoring.IsSequence() || scoring.size() == 0) {
    throw std::runtime_error("Config 'scoring' must be a non-empty array");
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
      active_trial_(std::nullopt),
      engine_state_(EngineState::Uninitialized) {
  RCLCPP_INFO(node_->get_logger(), "Creating AIC Engine...");

  // Declare ROS parameters.
  adapter_node_name_ = node_->declare_parameter(
      "adapter_node_name", std::string("aic_adapter_node"));
  model_node_name_ = node_->declare_parameter("model_node_name",
                                              std::string("aic_model_node"));
  node_->declare_parameter("config_file_path", std::string(""));
  node_->declare_parameter("endpoint_discovery_timeout_seconds", 10);

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
  insert_cable_action_client_ =
      rclcpp_action::create_client<InsertCableAction>(node_, "/insert_cable");
  spawn_entity_client_ =
      node_->create_client<SpawnEntitySrv>("/gz_server/spawn_entity");
  delete_entity_client_ =
      node_->create_client<DeleteEntitySrv>("/gz_server/delete_entity");

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

  if (!this->check_required_endpoints()) {
    RCLCPP_ERROR(node_->get_logger(), "Required endpoints are not available.");
    current_state = TrialState::Uninitialized;
    reset_after_trial();
    return current_state;
  }
  current_state = TrialState::EndpointsAvailable;

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

//==============================================================================
bool Engine::check_required_endpoints() {
  RCLCPP_INFO(node_->get_logger(), "Checking required endpoints...");

  // Check nodes
  bool all_available = false;
  rclcpp::Time start_time = this->node_->now();
  const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(
      this->node_->get_parameter("endpoint_discovery_timeout_seconds")
          .as_int());
  const auto& node_graph = node_->get_node_graph_interface();

  while (!all_available && !(this->node_->now() - start_time > timeout)) {
    all_available = true;
    const std::vector<std::string> graph_nodes = node_graph->get_node_names();
    std::unordered_set<std::string> node_set(graph_nodes.begin(),
                                             graph_nodes.end());
    for (const auto& node_name : REQUIRED_NODES) {
      all_available = all_available && (node_set.count(node_name) > 0);
    }
  }
  if (!all_available) {
    RCLCPP_ERROR(node_->get_logger(), "Not all required nodes are available.");
    return false;
  }

  // Check topics
  // TODO(Yadunund): Consider checking for messages received on topics.
  all_available = false;
  start_time = this->node_->now();
  while (!all_available && !(this->node_->now() - start_time > timeout)) {
    all_available = true;
    all_available = all_available &&
                    (this->wrench_sub_->get_publisher_count() > 0) &&
                    (this->joint_state_sub_->get_publisher_count() > 0);
  }
  if (!all_available) {
    RCLCPP_ERROR(node_->get_logger(), "Not all required topics are available.");
    return false;
  }

  // Check services
  if (!spawn_entity_client_->wait_for_service(
          timeout.to_chrono<std::chrono::nanoseconds>())) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Spawn entity service not available after waiting");
    return false;
  }

  // Check actioins
  // TODO(Yadunund): Re-enable action server check aic_model is implemented.
  // if (!insert_cable_action_client_->wait_for_action_server(timeout)) {
  // 	RCLCPP_ERROR(node_->get_logger(),
  // 								"Insert cable
  // action server not available after waiting"); 	return false;
  // }

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
  if (this->spawn_task_board(task_board_config["pose"]["x"].as<double>(),
                             task_board_config["pose"]["y"].as<double>(),
                             task_board_config["pose"]["z"].as<double>(),
                             task_board_config["pose"]["roll"].as<double>(),
                             task_board_config["pose"]["pitch"].as<double>(),
                             task_board_config["pose"]["yaw"].as<double>())) {
    RCLCPP_INFO(node_->get_logger(), "Task board spawned successfully.");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to spawn task board.");
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

void Engine::reset_after_trial() {
  RCLCPP_INFO(node_->get_logger(), "Resetting after trial completion...");

  // Remove spawned task board from simulator
  if (active_trial_.has_value() &&
      active_trial_->spawned_task_board_name.has_value()) {
    // Delete spawned task board
    auto request = std::make_shared<DeleteEntitySrv::Request>();
    request->entity = active_trial_->spawned_task_board_name.value();

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

  RCLCPP_INFO(node_->get_logger(), "Reset after trial completed.");
}

//==============================================================================
Engine::~Engine() {
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

//==============================================================================
bool Engine::spawn_task_board(double x, double y, double z, double roll,
                              double pitch, double yaw) {
  // Get the task board xacro file path
  const std::string aic_description_share =
      ament_index_cpp::get_package_share_directory("aic_description");
  const std::string xacro_file =
      aic_description_share + "/urdf/task_board.urdf.xacro";

  // Convert xacro to URDF using xacro command (without pose args)
  std::stringstream cmd;
  cmd << "xacro " << xacro_file;

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
  request->name = "task_board";
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
    RCLCPP_ERROR(node_->get_logger(), "Failed to spawn task board: %s",
                 response->result.error_message.c_str());
    return false;
  }

  if (active_trial_.has_value()) {
    active_trial_->spawned_task_board_name = response->entity_name;
  }

  RCLCPP_INFO(node_->get_logger(), "Successfully spawned task board as '%s'",
              response->entity_name.c_str());
  return true;
}

}  // namespace aic
