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

#include "lifecycle_transition_skill.h"

#include <chrono>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_transition_skill.pb.h"
#include "rclcpp/rclcpp.hpp"

namespace {

constexpr double kDefaultTimeoutMs = 60000.0;

class InitRos {
 public:
  InitRos() { rclcpp::init(0, nullptr); }
  ~InitRos() { rclcpp::shutdown(); }
};

class LifecycleClientNode : public rclcpp::Node {
 public:
  LifecycleClientNode() : Node("lifecycle_transition_skill_node") {}

  absl::StatusOr<bool> ChangeState(const std::string& node_name,
                                   uint8_t transition, double timeout_ms) {
    std::string service_name = "/" + node_name + "/change_state";
    auto client =
        this->create_client<lifecycle_msgs::srv::ChangeState>(service_name);

    if (!client->wait_for_service(std::chrono::seconds(5))) {
      return absl::UnavailableError("Service '" + service_name +
                                    "' not available");
    }

    auto request =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    auto future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future,
            std::chrono::milliseconds(static_cast<int>(timeout_ms))) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return absl::DeadlineExceededError(
          "Timed out waiting for change_state response");
    }

    auto response = future.get();
    return response->success;
  }

  absl::StatusOr<uint8_t> GetState(const std::string& node_name,
                                   double timeout_ms) {
    std::string service_name = "/" + node_name + "/get_state";
    auto client =
        this->create_client<lifecycle_msgs::srv::GetState>(service_name);

    if (!client->wait_for_service(std::chrono::seconds(5))) {
      return absl::UnavailableError("Service '" + service_name +
                                    "' not available");
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    auto future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future,
            std::chrono::milliseconds(static_cast<int>(timeout_ms))) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return absl::DeadlineExceededError(
          "Timed out waiting for get_state response");
    }

    auto response = future.get();
    return response->current_state.id;
  }
};

InitRos init;
LifecycleClientNode client_node_;

}  // namespace

std::unique_ptr<intrinsic::skills::SkillInterface>
LifecycleTransitionSkill::CreateSkill() {
  return std::make_unique<LifecycleTransitionSkill>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
LifecycleTransitionSkill::Preview(
    const intrinsic::skills::PreviewRequest& /*request*/,
    intrinsic::skills::PreviewContext& /*context*/) {
  return absl::UnimplementedError("Preview not supported for this skill");
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
LifecycleTransitionSkill::Execute(
    const intrinsic::skills::ExecuteRequest& request,
    intrinsic::skills::ExecuteContext& /*context*/) {
  INTR_ASSIGN_OR_RETURN(
      auto params,
      request.params<ai::flowstate::LifecycleTransitionSkillParams>());

  RCLCPP_INFO(client_node_.get_logger(),
              "Transitioning node %s to transition %d",
              params.node_name().c_str(), params.transition());

  uint8_t transition_id = 0;
  switch (static_cast<int>(params.transition())) {
    case 0:  // create
      transition_id = 0;
      break;
    case 1:  // configure
      transition_id = 1;
      break;
    case 2:  // cleanup
      transition_id = 2;
      break;
    case 3:  // activate
      transition_id = 3;
      break;
    case 4:  // deactivate
      transition_id = 4;
      break;
    case 5:  // shutdown
    {
      auto state_or_err = client_node_.GetState(params.node_name(), 5000.0);
      if (!state_or_err.ok()) {
        return state_or_err.status();
      }
      uint8_t current_state = state_or_err.value();
      if (current_state == 1) {         // PRIMARY_STATE_UNCONFIGURED
        transition_id = 5;              // TRANSITION_UNCONFIGURED_SHUTDOWN
      } else if (current_state == 2) {  // PRIMARY_STATE_INACTIVE
        transition_id = 6;              // TRANSITION_INACTIVE_SHUTDOWN
      } else if (current_state == 3) {  // PRIMARY_STATE_ACTIVE
        transition_id = 7;              // TRANSITION_ACTIVE_SHUTDOWN
      } else {
        return absl::FailedPreconditionError(
            "Cannot shutdown node from current state");
      }
    } break;
    case 6:  // destroy
      transition_id = 8;
      break;
    default:
      return absl::InvalidArgumentError(
          absl::StrCat("!!! Unknown transition value: ",
                       static_cast<int>(params.transition()), " !!!"));
  }

  double timeout_ms =
      params.timeout() > 0.0 ? params.timeout() * 1000.0 : kDefaultTimeoutMs;
  auto status_or_success =
      client_node_.ChangeState(params.node_name(), transition_id, timeout_ms);

  if (!status_or_success.ok()) {
    return status_or_success.status();
  }

  auto result =
      std::make_unique<ai::flowstate::LifecycleTransitionSkillResult>();
  result->set_success(status_or_success.value());
  if (!status_or_success.value()) {
    result->set_message("Transition failed");
  } else {
    result->set_message("Transition succeeded");
  }

  return result;
}
