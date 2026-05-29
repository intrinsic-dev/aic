// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fstream>

#include "absl/log/log.h"
#include "aic_flowstate_ros_bridge.pb.h"
#include "class_loader/class_loader.hpp"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

///=============================================================================
intrinsic_proto::config::RuntimeContext GetRuntimeContext() {
  intrinsic_proto::config::RuntimeContext runtime_context;
  std::ifstream runtime_context_file;
  runtime_context_file.open("/etc/intrinsic/runtime_config.pb",
                            std::ios::binary);
  if (!runtime_context.ParseFromIstream(&runtime_context_file)) {
    // Return default context for running locally
    std::cerr << "Warning: using default RuntimeContext\n";
  }
  return runtime_context;
}

///=============================================================================
int main(int argc, char* argv[]) {
  auto runtime_context = GetRuntimeContext();
  intrinsic::FlowstateRosBridgeConfig ros_config;
  if (!runtime_context.config().UnpackTo(&ros_config)) {
    LOG(WARNING) << "Error unpacking FlowstateRosBridgeConfig from service "
                    "config file... Passing empty ros args to node";
  }

  // Handle optional external router address
  std::string external_router_address;
  if (!ros_config.external_zenoh_router_address().empty()) {
    external_router_address = ros_config.external_zenoh_router_address();
  }

  // If external router address is provided, override the Zenoh environment
  // variable
  std::string zenoh_config_override = "connect/endpoints=[\"";
  if (!external_router_address.empty()) {
    zenoh_config_override += external_router_address;
  } else {
    // If no external address, look for "flowstate_zenoh_router_address" param
    zenoh_config_override += ros_config.flowstate_zenoh_router_address();
  }
  zenoh_config_override += "\"]";
  setenv("ZENOH_CONFIG_OVERRIDE", zenoh_config_override.c_str(), 1);
  LOG(INFO) << "ZENOH_CONFIG_OVERRIDE: " << zenoh_config_override;

  rclcpp::init(argc, argv);

  rclcpp::experimental::executors::EventsExecutor exec;
  rclcpp::NodeOptions options;

  std::vector<rclcpp::Parameter> params;
  // Get parameters from config
  params.emplace_back("executive_service_address",
                      ros_config.executive_service_address());
  params.emplace_back("executive_deadline_seconds",
                      ros_config.executive_deadline_seconds());
  params.emplace_back("executive_update_rate_millis",
                      ros_config.executive_update_rate_millis());
  params.emplace_back("skill_registry_address",
                      ros_config.skill_registry_address());
  params.emplace_back("solution_service_address",
                      ros_config.solution_service_address());
  params.emplace_back("world_service_address",
                      ros_config.world_service_address());
  params.emplace_back("geometry_service_address",
                      ros_config.geometry_service_address());
  params.emplace_back("flowstate_zenoh_router_address",
                      ros_config.flowstate_zenoh_router_address());
  const auto& bridge_plugins_proto = ros_config.bridge_plugins();
  std::vector<std::string> plugin_list(bridge_plugins_proto.begin(),
                                       bridge_plugins_proto.end());
  rclcpp::Parameter bridge_plugins_param("bridge_plugins", plugin_list);
  params.push_back(std::move(bridge_plugins_param));

  const auto& s = ros_config.sensors();
  params.emplace_back("enable_robot_joint_state_topic",
                      s.enable_robot_joint_state_topic());
  params.emplace_back("enable_force_torque_topic",
                      s.enable_force_torque_topic());
  params.emplace_back("robot_joint_state_topic", s.robot_joint_state_topic());
  params.emplace_back("force_torque_topic", s.force_torque_topic());
  params.emplace_back("force_torque_sensor_frame_id",
                      s.force_torque_sensor_frame_id());
  params.emplace_back("robot_controller_instance",
                      s.robot_controller_instance());
  params.emplace_back("throttle_robot_state_topic",
                      s.throttle_robot_state_topic());

  const auto& robot_control_bridge_config =
      ros_config.robot_control_bridge_config();
  params.emplace_back("server_address",
                      robot_control_bridge_config.server_address());
  params.emplace_back("instance", robot_control_bridge_config.instance());
  params.emplace_back("part_name", robot_control_bridge_config.part_name());
  params.emplace_back("ft_sensor_part_name",
                      robot_control_bridge_config.ft_sensor_part_name());
  params.emplace_back("task_settings_file",
                      robot_control_bridge_config.task_settings_file());
  params.emplace_back("joint_task_settings_file",
                      robot_control_bridge_config.joint_task_settings_file());
  const auto& override_joint_names_proto = s.override_joint_names();
  std::vector<std::string> override_joint_names(
      override_joint_names_proto.begin(), override_joint_names_proto.end());
  params.emplace_back("override_joint_names", override_joint_names);

  options.parameter_overrides(params);

  // Get namespace from config
  std::vector<std::string> remap_rules;
  remap_rules.push_back("--ros-args");
  if (ros_config.workcell_id() != "") {
    remap_rules.push_back("-r");
    remap_rules.push_back("__ns:=/" + ros_config.workcell_id());
  }
  options.arguments(remap_rules);

  // Create and spin the FlowstateROSBridge node
  // Adapted from rclcpp_components::node_main.cpp.in
  std::string library_name = "libflowstate_ros_bridge_component.so";
  std::string class_name =
      "rclcpp_components::NodeFactoryTemplate<flowstate_ros_bridge::"
      "FlowstateROSBridge>";

  LOG(INFO) << "Load library " << library_name;
  auto loader = std::make_unique<class_loader::ClassLoader>(library_name);
  std::vector<std::string> classes =
      loader->getAvailableClasses<rclcpp_components::NodeFactory>();

  if (std::find(classes.begin(), classes.end(), class_name) == classes.end()) {
    LOG(INFO) << "Class " << class_name << " not found in library "
              << library_name;
    return 1;
  }
  LOG(INFO) << "Instantiate class " << class_name;
  std::shared_ptr<rclcpp_components::NodeFactory> node_factory = nullptr;
  try {
    node_factory =
        loader->createInstance<rclcpp_components::NodeFactory>(class_name);
  } catch (const std::exception& ex) {
    LOG(ERROR) << "Failed to load library " << ex.what();
    return 1;
  } catch (...) {
    LOG(ERROR) << "Failed to load library";
    return 1;
  }
  // Scope to destruct node_wrapper before shutdown
  {
    rclcpp_components::NodeInstanceWrapper node_wrapper =
        node_factory->create_node_instance(options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node =
        node_wrapper.get_node_base_interface();
    exec.add_node(node);

    exec.spin();

    exec.remove_node(node_wrapper.get_node_base_interface());
  }

  // Shutdown ROS
  rclcpp::shutdown();

  return 0;
}
