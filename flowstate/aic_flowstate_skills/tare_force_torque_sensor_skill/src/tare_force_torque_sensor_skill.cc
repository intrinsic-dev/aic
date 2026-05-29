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

#include "tare_force_torque_sensor_skill.h"

#include <chrono>

#include "absl/status/statusor.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tare_force_torque_sensor_skill.pb.h"

//==============================================================================
// ROS initialization.
//==============================================================================

namespace {
class InitRos {
 public:
  InitRos() { rclcpp::init(0, nullptr); }
  ~InitRos() { rclcpp::shutdown(); }
};

class TareFTClientNode : public rclcpp::Node {
 public:
  TareFTClientNode() : Node("tare_force_torque_sensor_skill_node") {
    client_ = this->create_client<std_srvs::srv::Trigger>(
        "aic_controller/tare_force_torque_sensor");
  }

  absl::StatusOr<std_srvs::srv::Trigger::Response::SharedPtr> CallService(
      double timeout_ms) {
    if (!client_->wait_for_service(std::chrono::seconds(10))) {
      return absl::UnavailableError(
          "Service 'aic_controller/tare_force_torque_sensor' not available");
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
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

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

InitRos init;
TareFTClientNode client_node_;

}  // namespace

#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"

//==============================================================================
// Skill signature.
//==============================================================================

std::unique_ptr<intrinsic::skills::SkillInterface>
TareForceTorqueSensorSkill::CreateSkill() {
  return std::make_unique<TareForceTorqueSensorSkill>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
TareForceTorqueSensorSkill::Preview(
    const intrinsic::skills::PreviewRequest& /*request*/,
    intrinsic::skills::PreviewContext& /*context*/) {
  return absl::UnimplementedError("Skill has not implemented `Preview`.");
}

//==============================================================================
// Skill execution.
//==============================================================================

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
TareForceTorqueSensorSkill::Execute(
    const intrinsic::skills::ExecuteRequest& request,
    intrinsic::skills::ExecuteContext& /*context*/) {
  RCLCPP_INFO(client_node_.get_logger(), "TareForceTorqueSensorSkill::Execute");

  INTR_ASSIGN_OR_RETURN(
      auto params,
      request.params<ai::flowstate::TareForceTorqueSensorSkillParams>());

  auto timeout_ms =
      params.time_limit() > 0 ? params.time_limit() * 1000.0 : 10000.0;

  auto status_or_result = client_node_.CallService(timeout_ms);

  if (!status_or_result.ok()) {
    return status_or_result.status();
  }

  auto service_result = status_or_result.value();

  auto result =
      std::make_unique<ai::flowstate::TareForceTorqueSensorSkillResult>();
  result->set_success(service_result->success);
  result->set_message(service_result->message);

  return result;
}
