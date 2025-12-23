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

#include <filesystem>

#include "aic_task_interfaces/msg/task.hpp"

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
}

}  // namespace aic

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(aic::Engine)
