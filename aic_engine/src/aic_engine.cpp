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

#include "aic_task_interfaces/msg/task.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace aic {

//==============================================================================
Trial::Trial(const std::string& id, const std::string& cable_type,
             const std::string& cable_name, const std::string& plug_type,
             const std::string& plug_name, const std::string& port_type,
             const std::string& port_name,
             const std::string& target_module_name, std::size_t time_limit) {
  task = aic_task_interfaces::build<aic_task_interfaces::msg::Task>()
             .id(id)
             .cable_type(cable_type)
             .cable_name(cable_name)
             .plug_type(plug_type)
             .plug_name(plug_name)
             .port_type(port_type)
             .port_name(port_name)
             .target_module_name(target_module_name)
             .time_limit(time_limit);

  state = TrialState::Uninitialized;
}

//==============================================================================
Engine::Engine(const rclcpp::NodeOptions& options)
    : rclcpp::Node("aic_engine", options) {
  RCLCPP_INFO(this->get_logger(), "Starting AIC Engine...");

  // Declare ROS parameters.
  adapter_node_name_ = this->declare_parameter("adapter_node_name",
                                               std::string("aic_adapter_node"));
  model_node_name_ =
      this->declare_parameter("model_node_name", std::string("aic_model_node"));
  const std::filesystem::path config_file_path =
      this->declare_parameter("config_file_path", std::string(""));
  // If file path is valid, load contents into config_ as YAML.
  // Note: exception will be thrown if file is not found or invalid YAML.
  config_ = YAML::LoadFile(config_file_path);

  insert_cable_action_client_ =
      rclcpp_action::create_client<InsertCableAction>(this, "/insert_cable");

  spawn_entity_client_ =
      this->create_client<SpawnEntitySrv>("/gz_server/spawn_entity");

  RCLCPP_INFO(this->get_logger(), "AIC Engine started.");

  // TODO(Yadunund): Remove this temporary code for spawning the task board.
  task_thread_ = std::thread([this]() {
    // TODO(Yadunund): Implement state machine logic to manage stages of
    // different trials. For now, just spawn the task board after a delay.
    std::this_thread::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(this->get_logger(), "Spawning task board.");
    const auto& task_board_config =
        config_["trials"]["trial_1"]["scene"]["task_board"];
    if (this->spawn_task_board(task_board_config["pose"]["x"].as<double>(),
                               task_board_config["pose"]["y"].as<double>(),
                               task_board_config["pose"]["z"].as<double>(),
                               task_board_config["pose"]["roll"].as<double>(),
                               task_board_config["pose"]["pitch"].as<double>(),
                               task_board_config["pose"]["yaw"].as<double>())) {
      RCLCPP_INFO(this->get_logger(), "Task board spawned successfully.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to spawn task board.");
    }
  });
}

//==============================================================================
Engine::~Engine() {
  if (task_thread_.joinable()) {
    task_thread_.join();
  }
}

//==============================================================================
bool Engine::spawn_task_board(double x, double y, double z, double roll,
                              double pitch, double yaw) {
  if (!spawn_entity_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Spawn entity service not available after waiting");
    return false;
  }

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
    RCLCPP_ERROR(this->get_logger(), "Failed to execute xacro command");
    return false;
  }

  std::stringstream urdf_stream;
  char buffer[128];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    urdf_stream << buffer;
  }
  int result = pclose(pipe);
  if (result != 0) {
    RCLCPP_ERROR(this->get_logger(), "xacro command failed with code %d",
                 result);
    return false;
  }

  std::string urdf_string = urdf_stream.str();
  if (urdf_string.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Generated URDF is empty");
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

  // Call service asynchronously
  auto future = spawn_entity_client_->async_send_request(request);

  // Wait for the response with a timeout
  auto status = future.wait_for(std::chrono::seconds(10));
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call spawn entity service");
    return false;
  }

  auto response = future.get();
  if (response->result.result != 1) {  // RESULT_OK = 1
    RCLCPP_ERROR(this->get_logger(), "Failed to spawn task board: %s",
                 response->result.error_message.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully spawned task board as '%s'",
              response->entity_name.c_str());
  return true;
}

}  // namespace aic

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(aic::Engine)
