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

#include "insert_cable_skill.h"

#include <chrono>

#include "absl/status/statusor.h"
#include "aic_task_interfaces/action/insert_cable.hpp"
#include "insert_cable_skill.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using InsertCableAction = aic_task_interfaces::action::InsertCable;

//==============================================================================
// ROS initialization.
//==============================================================================

namespace {
class InitRos {
 public:
  InitRos() { rclcpp::init(0, nullptr); }
  ~InitRos() { rclcpp::shutdown(); }
};

std::mutex active_goal_mutex;
typename rclcpp_action::Client<InsertCableAction>::GoalHandle::SharedPtr
    active_goal_handle;

class InsertCableClientNode : public rclcpp::Node {
 public:
  InsertCableClientNode() : Node("insert_cable_skill_node") {
    client_ =
        rclcpp_action::create_client<InsertCableAction>(this, "/insert_cable");
  }

  absl::StatusOr<InsertCableAction::Result::SharedPtr> SendAction(
      const InsertCableAction::Goal& goal, double timeout_ms) {
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      return absl::UnavailableError(
          "Action server '/insert_cable' not available");
    }

    auto send_goal_options =
        rclcpp_action::Client<InsertCableAction>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](
            const rclcpp_action::ClientGoalHandle<InsertCableAction>::SharedPtr&
                goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
          } else {
            RCLCPP_INFO(this->get_logger(),
                        "Goal accepted by server, waiting for result");
          }
        };

    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<
               InsertCableAction>::WrappedResult& result) {
          switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(this->get_logger(), "Goal succeeded");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
              break;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
              break;
            default:
              RCLCPP_ERROR(this->get_logger(), "Unknown result code");
              break;
          }
        };

    RCLCPP_INFO(this->get_logger(), "Sending goal to /insert_cable");
    auto goal_handle_future = client_->async_send_goal(goal, send_goal_options);

    if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), goal_handle_future,
            std::chrono::milliseconds(static_cast<int>(timeout_ms))) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return absl::DeadlineExceededError(
          "Timed out waiting for goal response.");
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      return absl::InternalError("Goal was rejected by server.");
    }

    // Store active handle thread-safely for concurrent cancellation halts
    {
      std::lock_guard<std::mutex> lock(active_goal_mutex);
      active_goal_handle = goal_handle;
    }

    auto result_future = client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), result_future,
            std::chrono::milliseconds(static_cast<int>(timeout_ms))) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      // Clear active handle on timeout
      {
        std::lock_guard<std::mutex> lock(active_goal_mutex);
        active_goal_handle.reset();
      }
      return absl::DeadlineExceededError(
          "Timed out waiting for action result.");
    }

    // Clear active handle
    {
      std::lock_guard<std::mutex> lock(active_goal_mutex);
      active_goal_handle.reset();
    }

    auto result_wrapper = result_future.get();
    if (result_wrapper.code != rclcpp_action::ResultCode::SUCCEEDED) {
      return absl::InternalError("Action failed or was canceled.");
    }

    return result_wrapper.result;
  }

  rclcpp_action::Client<InsertCableAction>::SharedPtr client_;
};

InitRos init;
InsertCableClientNode client_node_;

}  // namespace

#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

//==============================================================================
// Skill signature.
//==============================================================================

std::unique_ptr<intrinsic::skills::SkillInterface>
InsertCableSkill::CreateSkill() {
  return std::make_unique<InsertCableSkill>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
InsertCableSkill::Preview(const intrinsic::skills::PreviewRequest& /*request*/,
                          intrinsic::skills::PreviewContext& /*context*/) {
  return absl::UnimplementedError("Skill has not implemented `Preview`.");
}

//==============================================================================
// Skill execution.
//==============================================================================

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
InsertCableSkill::Execute(const intrinsic::skills::ExecuteRequest& request,
                          intrinsic::skills::ExecuteContext& /*context*/) {
  RCLCPP_INFO(client_node_.get_logger(), "InsertCableSkill::Execute");

  INTR_ASSIGN_OR_RETURN(
      auto params, request.params<ai::flowstate::InsertCableSkillParams>());

  // Populate goal msg
  auto goal_msg = InsertCableAction::Goal();
  goal_msg.task.id = params.id();
  goal_msg.task.cable_type = params.cable_type();
  goal_msg.task.cable_name = params.cable_name();
  goal_msg.task.plug_type = params.plug_type();
  goal_msg.task.plug_name = params.plug_name();
  goal_msg.task.port_type = params.port_type();
  goal_msg.task.port_name = params.port_name();
  goal_msg.task.target_module_name = params.target_module_name();
  goal_msg.task.time_limit = params.time_limit();

  RCLCPP_INFO(client_node_.get_logger(), "Sending goal for task ID: %s",
              goal_msg.task.id.c_str());

  auto status_or_result =
      client_node_.SendAction(goal_msg, params.time_limit() * 1000.0);

  if (!status_or_result.ok()) {
    return status_or_result.status();
  }

  auto action_result = status_or_result.value();

  auto result = std::make_unique<ai::flowstate::InsertCableSkillResult>();
  result->set_success(action_result->success);
  result->set_message(action_result->message);

  return result;
}
