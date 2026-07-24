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

#include "start_recording_skill.h"

#include <chrono>

#include "absl/status/statusor.h"
#include "aic_logger_interfaces/srv/start_recording.hpp"
#include "rclcpp/rclcpp.hpp"
#include "start_recording_skill.pb.h"

//==============================================================================
// ROS initialization.
//==============================================================================

namespace {
class InitRos {
 public:
  InitRos() { rclcpp::init(0, nullptr); }
  ~InitRos() { rclcpp::shutdown(); }
};

class StartRecordingClientNode : public rclcpp::Node {
 public:
  StartRecordingClientNode() : Node("start_recording_skill_node") {
    client_ = this->create_client<aic_logger_interfaces::srv::StartRecording>(
        "/aic_logger/start_recording");
  }

  absl::StatusOr<aic_logger_interfaces::srv::StartRecording::Response::SharedPtr>
  CallService(
      const aic_logger_interfaces::srv::StartRecording::Request::SharedPtr& request,
      double timeout_ms) {
    if (!client_->wait_for_service(std::chrono::seconds(10))) {
      return absl::UnavailableError("Service '/aic_logger/start_recording' not available");
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

  rclcpp::Client<aic_logger_interfaces::srv::StartRecording>::SharedPtr client_;
};

InitRos init;
StartRecordingClientNode client_node_;

}  // namespace

#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"

//==============================================================================
// Skill signature.
//==============================================================================

std::unique_ptr<intrinsic::skills::SkillInterface>
StartRecordingSkill::CreateSkill() {
  return std::make_unique<StartRecordingSkill>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
StartRecordingSkill::Preview(const intrinsic::skills::PreviewRequest& /*request*/,
                            intrinsic::skills::PreviewContext& /*context*/) {
  return absl::UnimplementedError("Skill has not implemented `Preview`.");
}

//==============================================================================
// Skill execution.
//==============================================================================

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
StartRecordingSkill::Execute(const intrinsic::skills::ExecuteRequest& request,
                            intrinsic::skills::ExecuteContext& /*context*/) {
  RCLCPP_INFO(client_node_.get_logger(), "StartRecordingSkill::Execute");

  INTR_ASSIGN_OR_RETURN(
      auto params, request.params<ai::flowstate::StartRecordingSkillParams>());

  auto service_request =
      std::make_shared<aic_logger_interfaces::srv::StartRecording::Request>();
  service_request->name = params.name();

  RCLCPP_INFO(client_node_.get_logger(), "Starting recording (name: %s)...",
              params.name().c_str());

  const double timeout_ms = 30000.0;
  auto status_or_result = client_node_.CallService(service_request, timeout_ms);

  if (!status_or_result.ok()) {
    return status_or_result.status();
  }

  auto service_result = status_or_result.value();
  if (!service_result->success) {
    RCLCPP_ERROR(client_node_.get_logger(), "Failed starting recording: %s",
                 service_result->message.c_str());
    return absl::InternalError(service_result->message);
  }

  RCLCPP_INFO(client_node_.get_logger(), "Recording started successfully. Bag path: %s",
              service_result->bag_path.c_str());

  auto result = std::make_unique<ai::flowstate::StartRecordingSkillResult>();
  result->set_bag_path(service_result->bag_path);
  return result;
}
