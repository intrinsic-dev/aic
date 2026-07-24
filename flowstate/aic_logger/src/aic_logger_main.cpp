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
#include <iostream>
#include <memory>
#include <vector>

#include "absl/log/log.h"
#include "aic_logger.pb.h"
#include "aic_logger/aic_logger_node.hpp"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "rclcpp/rclcpp.hpp"

intrinsic_proto::config::RuntimeContext GetRuntimeContext() {
  intrinsic_proto::config::RuntimeContext runtime_context;
  std::ifstream runtime_context_file;
  runtime_context_file.open("/etc/intrinsic/runtime_config.pb",
                            std::ios::binary);
  if (!runtime_context.ParseFromIstream(&runtime_context_file)) {
    std::cerr << "Warning: using default RuntimeContext\n";
  }
  return runtime_context;
}

int main(int argc, char* argv[]) {
  auto runtime_context = GetRuntimeContext();
  intrinsic::AicLoggerConfig logger_config;
  if (!runtime_context.config().UnpackTo(&logger_config)) {
    LOG(WARNING) << "Error unpacking AicLoggerConfig from service config file...";
  }

  std::string zenoh_config_override = "connect/endpoints=[\"";
  if (!logger_config.flowstate_zenoh_router_address().empty()) {
    zenoh_config_override += logger_config.flowstate_zenoh_router_address();
  } else {
    zenoh_config_override += "tcp/zenoh-router.app-intrinsic-base.svc.cluster.local:7447";
  }
  zenoh_config_override += "\"]";
  setenv("ZENOH_CONFIG_OVERRIDE", zenoh_config_override.c_str(), 1);
  LOG(INFO) << "ZENOH_CONFIG_OVERRIDE: " << zenoh_config_override;

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  if (!logger_config.workcell_id().empty()) {
    std::vector<std::string> remap_rules;
    remap_rules.push_back("--ros-args");
    remap_rules.push_back("-r");
    remap_rules.push_back("__ns:=/" + logger_config.workcell_id());
    options.arguments(remap_rules);
  }

  auto node = std::make_shared<aic_logger::AicLoggerNode>(options);

  std::vector<std::string> topics(logger_config.topics().begin(),
                                  logger_config.topics().end());
  std::vector<std::string> services(logger_config.services().begin(),
                                    logger_config.services().end());
  std::vector<std::string> actions(logger_config.actions().begin(),
                                   logger_config.actions().end());

  node->Configure(logger_config.output_directory(),
                  logger_config.storage_id(),
                  topics,
                  services,
                  actions,
                  logger_config.record_all_topics(),
                  logger_config.record_all_services(),
                  logger_config.record_all_actions());

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
