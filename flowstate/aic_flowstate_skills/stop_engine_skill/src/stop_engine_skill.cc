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

#include "stop_engine_skill.h"

#include <chrono>

#include "absl/status/statusor.h"
#include "aic_engine_interfaces/srv/stop_engine.hpp"
#include "rclcpp/rclcpp.hpp"
#include "stop_engine_skill.pb.h"

//==============================================================================
// ROS initialization.
//==============================================================================

namespace {
class InitRos {
 public:
  InitRos() { rclcpp::init(0, nullptr); }
  ~InitRos() { rclcpp::shutdown(); }
};

class StopEngineClientNode : public rclcpp::Node {
 public:
  StopEngineClientNode() : Node("stop_engine_skill_node") {
    client_ = this->create_client<aic_engine_interfaces::srv::StopEngine>(
        "/stop_aic_engine");
  }

  absl::StatusOr<aic_engine_interfaces::srv::StopEngine::Response::SharedPtr>
  CallService(const aic_engine_interfaces::srv::StopEngine::Request::SharedPtr&
                  request,
              double timeout_ms) {
    if (!client_->wait_for_service(std::chrono::seconds(10))) {
      return absl::UnavailableError(
          "Service '/stop_aic_engine' not available");
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

  rclcpp::Client<aic_engine_interfaces::srv::StopEngine>::SharedPtr client_;
};

InitRos init;
StopEngineClientNode client_node_;

}  // namespace

#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"

//==============================================================================
// Skill signature.
//==============================================================================

std::unique_ptr<intrinsic::skills::SkillInterface>
StopEngineSkill::CreateSkill() {
  return std::make_unique<StopEngineSkill>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
StopEngineSkill::Preview(const intrinsic::skills::PreviewRequest& /*request*/,
                          intrinsic::skills::PreviewContext& /*context*/) {
  return absl::UnimplementedError("Skill has not implemented `Preview`.");
}

//==============================================================================
// Skill execution.
//==============================================================================

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
StopEngineSkill::Execute(const intrinsic::skills::ExecuteRequest& request,
                          intrinsic::skills::ExecuteContext& /*context*/) {
  RCLCPP_INFO(client_node_.get_logger(), "StopEngineSkill::Execute");

  INTR_ASSIGN_OR_RETURN(auto params,
                        request.params<ai::flowstate::StopEngineSkillParams>());

  // Populate service request
  auto service_request =
      std::make_shared<aic_engine_interfaces::srv::StopEngine::Request>();
  service_request->task_id = params.task_id();
  service_request->finished = params.finished();

  RCLCPP_INFO(client_node_.get_logger(), "Stopping engine for task ID: %s",
              service_request->task_id.c_str());

  // Default timeout of 60s
  const double timeout_ms = 60000.0;

  auto status_or_result = client_node_.CallService(service_request, timeout_ms);

  if (!status_or_result.ok()) {
    return status_or_result.status();
  }

  auto service_result = status_or_result.value();
  if (!service_result->success) {
    return absl::InternalError(service_result->message);
  }

  return std::make_unique<ai::flowstate::StopEngineSkillResult>();
}
