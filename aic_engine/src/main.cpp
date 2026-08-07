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

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "aic_engine.hpp"
#include "aic_engine.pb.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

//==============================================================================
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

//==============================================================================
int main(int argc, char** argv) {
  auto runtime_context = GetRuntimeContext();
  intrinsic::AicEngineConfig config;
  bool use_sim_time = true;
  if (runtime_context.config().UnpackTo(&config)) {
    use_sim_time = config.use_sim_time();
  } else {
    std::cerr << "Warning: Error unpacking AicEngineConfig from service "
                 "config file... Passing default ros args to node\n";
  }

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("use_sim_time", use_sim_time);
  options.parameter_overrides(params);

  auto engine = std::make_shared<aic::Engine>(options);
  engine->spin();

  rclcpp::shutdown();
  return 0;
}
