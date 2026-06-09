/*
 * Copyright (C) 2026 Intrinsic Innovation LLC
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

#include "start_engine_skill.h"

#include <chrono>

#include "absl/status/statusor.h"
#include "aic_engine_interfaces/srv/start_engine.hpp"
#include "rclcpp/rclcpp.hpp"
#include "start_engine_skill.pb.h"

//==============================================================================
// ROS initialization.
//==============================================================================

namespace {
class InitRos {
 public:
  InitRos() { rclcpp::init(0, nullptr); }
  ~InitRos() { rclcpp::shutdown(); }
};

class StartEngineClientNode : public rclcpp::Node {
 public:
  StartEngineClientNode() : Node("start_engine_skill_node") {
    client_ = this->create_client<aic_engine_interfaces::srv::StartEngine>(
        "/start_aic_engine");
  }

  absl::StatusOr<aic_engine_interfaces::srv::StartEngine::Response::SharedPtr>
  CallService(const aic_engine_interfaces::srv::StartEngine::Request::SharedPtr&
                  request,
              double timeout_ms) {
    if (!client_->wait_for_service(std::chrono::seconds(10))) {
      return absl::UnavailableError(
          "Service '/start_aic_engine' not available");
    }

    auto result_future = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), result_future,
            std::chrono::milliseconds(static_cast<int>(timeout_ms))) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return absl::DeadlineExceededError(
          "Timed out waiting for service response.");
    }

    return result_future.get();
  }

  rclcpp::Client<aic_engine_interfaces::srv::StartEngine>::SharedPtr client_;
};

InitRos init;
StartEngineClientNode client_node_;

}  // namespace

#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"

//==============================================================================
// Skill signature.
//==============================================================================

std::unique_ptr<intrinsic::skills::SkillInterface>
StartEngineSkill::CreateSkill() {
  return std::make_unique<StartEngineSkill>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
StartEngineSkill::Preview(const intrinsic::skills::PreviewRequest& /*request*/,
                          intrinsic::skills::PreviewContext& /*context*/) {
  return absl::UnimplementedError("Skill has not implemented `Preview`.");
}

//==============================================================================
// Skill execution.
//==============================================================================

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
StartEngineSkill::Execute(const intrinsic::skills::ExecuteRequest& request,
                          intrinsic::skills::ExecuteContext& /*context*/) {
  RCLCPP_INFO(client_node_.get_logger(), "StartEngineSkill::Execute");

  INTR_ASSIGN_OR_RETURN(
      auto params, request.params<ai::flowstate::StartEngineSkillParams>());

  // Populate service request
  auto service_request =
      std::make_shared<aic_engine_interfaces::srv::StartEngine::Request>();
  if (params.plug_name().size() != 2 ||
      params.port_name().size() != 2 ||
      params.target_module_name().size() != 2) {
    return absl::InvalidArgumentError("Exactly two plug_name, port_name "
        "and target_module_name must be provided");
  }

  service_request->plug_name[0] = params.plug_name(0);
  service_request->plug_name[1] = params.plug_name(1);
  service_request->port_name[0] = params.port_name(0);
  service_request->port_name[1] = params.port_name(1);
  service_request->target_module_name[0] = params.target_module_name()[0];
  service_request->target_module_name[1] = params.target_module_name()[1];
  service_request->time_limit = params.time_limit();
  service_request->reset = params.reset();

  RCLCPP_INFO(client_node_.get_logger(), "Starting engine");

  // Use a default timeout of 60s
  const double timeout_ms = 60000;

  auto status_or_result = client_node_.CallService(service_request, timeout_ms);

  if (!status_or_result.ok()) {
    return status_or_result.status();
  }

  auto service_result = status_or_result.value();
  if (!service_result->success) {
    RCLCPP_INFO(
        client_node_.get_logger(), "Failed starting task, reason: %s",
        service_result->message.c_str());
    return absl::InternalError(service_result->message);
  }

  RCLCPP_INFO(client_node_.get_logger(), "Task started successfully");

  return std::make_unique<ai::flowstate::StartEngineSkillResult>();
}
