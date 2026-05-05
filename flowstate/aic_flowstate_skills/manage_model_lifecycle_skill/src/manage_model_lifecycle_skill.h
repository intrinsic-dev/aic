#pragma once

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/skills/cc/skill_interface.h"

class ManageModelLifecycleSkill : public intrinsic::skills::SkillInterface {
 public:
  ManageModelLifecycleSkill() = default;
  ~ManageModelLifecycleSkill() override = default;

  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> Execute(
      const intrinsic::skills::ExecuteRequest& request,
      intrinsic::skills::ExecuteContext& context) override;

  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> Preview(
      const intrinsic::skills::PreviewRequest& request,
      intrinsic::skills::PreviewContext& context) override;

  static absl::StatusOr<std::unique_ptr<intrinsic::skills::SkillInterface>>
  CreateSkill();
};
