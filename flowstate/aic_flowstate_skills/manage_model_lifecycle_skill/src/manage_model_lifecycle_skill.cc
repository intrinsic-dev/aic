#include "manage_model_lifecycle_skill.h"

#include <chrono>
#include <memory>
#include <mutex>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

#include "manage_model_lifecycle_skill.pb.h"

namespace {

class ManageModelLifecycleClientNode : public rclcpp::Node {
 public:
  ManageModelLifecycleClientNode() : Node("manage_model_lifecycle_skill_node") {
    client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
        "/aic_model/change_state");
  }

  absl::StatusOr<bool> ChangeState(uint8_t transition, double timeout_ms) {
    if (!client_->wait_for_service(std::chrono::seconds(5))) {
      return absl::UnavailableError(
          "Service '/aic_model/change_state' not available");
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    auto future = client_->async_send_request(request);
    
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

 private:
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;
};

static ManageModelLifecycleClientNode& GetNode() {
    static ManageModelLifecycleClientNode node;
    return node;
}

} // namespace

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
ManageModelLifecycleSkill::Execute(const intrinsic::skills::ExecuteRequest& request,
                                  intrinsic::skills::ExecuteContext& /*context*/) {
  RCLCPP_INFO(GetNode().get_logger(), "ManageModelLifecycleSkill::Execute");

  INTR_ASSIGN_OR_RETURN(
      auto params, request.params<ai::flowstate::ManageModelLifecycleSkillParams>());

  uint8_t transition = 0;
  switch (params.transition()) {
    case ai::flowstate::ManageModelLifecycleSkillParams::CONFIGURE:
      transition = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
      break;
    case ai::flowstate::ManageModelLifecycleSkillParams::ACTIVATE:
      transition = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
      break;
    case ai::flowstate::ManageModelLifecycleSkillParams::DEACTIVATE:
      transition = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
      break;
    case ai::flowstate::ManageModelLifecycleSkillParams::CLEANUP:
      transition = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
      break;
    case ai::flowstate::ManageModelLifecycleSkillParams::SHUTDOWN:
      transition = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
      break;
    default:
      return absl::InvalidArgumentError("Unknown transition");
  }

  auto status_or_success = GetNode().ChangeState(transition, 10000.0);

  if (!status_or_success.ok()) {
    return status_or_success.status();
  }

  auto result = std::make_unique<ai::flowstate::ManageModelLifecycleSkillResult>();
  result->set_success(status_or_success.value());
  if (status_or_success.value()) {
      result->set_message("Transition successful");
  } else {
      result->set_message("Transition failed on server");
  }

  return result;
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
ManageModelLifecycleSkill::Preview(const intrinsic::skills::PreviewRequest& /*request*/,
                                  intrinsic::skills::PreviewContext& /*context*/) {
  return absl::UnimplementedError("Preview not supported for this skill");
}

absl::StatusOr<std::unique_ptr<intrinsic::skills::SkillInterface>>
ManageModelLifecycleSkill::CreateSkill() {
  return std::make_unique<ManageModelLifecycleSkill>();
}
