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
#include <memory>
#include <string>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "google/protobuf/message.h"
#include "lifecycle_transition_skill.pb.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp/rclcpp.hpp"

// Assuming INTR_ASSIGN_OR_RETURN is available via some included header or transitively.
// If not, we might need to include "intrinsic/util/status/status_macros.h" if it exists.

namespace {

class InitRos {
 public:
  InitRos() { rclcpp::init(0, nullptr); }
  ~InitRos() { rclcpp::shutdown(); }
};

InitRos init;

class LifecycleTransitionClientNode : public rclcpp::Node {
 public:
  LifecycleTransitionClientNode() : Node("lifecycle_transition_skill_node") {}
};

std::shared_ptr<LifecycleTransitionClientNode> client_node_;

std::shared_ptr<LifecycleTransitionClientNode> GetClientNode() {
  if (!client_node_) {
    client_node_ = std::make_shared<LifecycleTransitionClientNode>();
  }
  return client_node_;
}

uint8_t TransitionStringToId(const std::string& transition) {
  if (transition == "configure") {
    return lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
  } else if (transition == "cleanup") {
    return lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
  } else if (transition == "activate") {
    return lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
  } else if (transition == "deactivate") {
    return lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
  } else if (transition == "active_shutdown") {
    return lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN;
  } else if (transition == "inactive_shutdown") {
    return lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN;
  } else if (transition == "unconfigured_shutdown") {
    return lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
  }
  return 0; // Unknown
}

}  // namespace

std::unique_ptr<intrinsic::skills::SkillInterface>
LifecycleTransitionSkill::CreateSkill() {
  return std::make_unique<LifecycleTransitionSkill>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
LifecycleTransitionSkill::Preview(const intrinsic::skills::PreviewRequest& /*request*/,
                                  intrinsic::skills::PreviewContext& /*context*/) {
  return absl::UnimplementedError("Skill has not implemented `Preview`.");
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
LifecycleTransitionSkill::Execute(const intrinsic::skills::ExecuteRequest& request,
                                  intrinsic::skills::ExecuteContext& /*context*/) {
  auto node = GetClientNode();
  RCLCPP_INFO(node->get_logger(), "LifecycleTransitionSkill::Execute");

  // We use a manual approach if INTR_ASSIGN_OR_RETURN fails to compile,
  // but let's try to use it assuming it works like in insert_cable_skill.cc.
  
  auto params_or = request.params<ai::flowstate::LifecycleTransitionParams>();
  if (!params_or.ok()) {
    return params_or.status();
  }
  auto params = params_or.value();

  const std::string& node_name = params.node_name();
  const std::string& target_state = params.target_state();

  RCLCPP_INFO(node->get_logger(), "Transitioning node %s to %s",
              node_name.c_str(), target_state.c_str());

  uint8_t transition_id = TransitionStringToId(target_state);
  if (transition_id == 0) {
    return absl::InvalidArgumentError("Unknown transition: " + target_state);
  }

  std::string service_name = node_name + "/change_state";
  auto client = node->create_client<lifecycle_msgs::srv::ChangeState>(service_name);

  auto timeout_sec = params.timeout();
  if (timeout_sec == 0) {
      timeout_sec = 5; // Default timeout
  }

  if (!client->wait_for_service(std::chrono::seconds(timeout_sec))) {
    return absl::UnavailableError("Service not available: " + service_name);
  }

  auto srv_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  srv_request->transition.id = transition_id;
  srv_request->transition.label = target_state;

  auto future = client->async_send_request(srv_request);

  if (rclcpp::spin_until_future_complete(
          node->get_node_base_interface(), future,
          std::chrono::seconds(timeout_sec)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return absl::DeadlineExceededError("Timed out waiting for change_state response");
  }

  auto response = future.get();
  if (!response->success) {
    return absl::InternalError("Lifecycle transition failed for node " + node_name);
  }

  RCLCPP_INFO(node->get_logger(), "Successfully transitioned node %s", node_name.c_str());

  auto result = std::make_unique<ai::flowstate::LifecycleTransitionResult>();
  result->set_success(true);
  result->set_message("Successfully transitioned node " + node_name);

  return result;
}
