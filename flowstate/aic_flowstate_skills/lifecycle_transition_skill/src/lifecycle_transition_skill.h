#ifndef LIFECYCLE_TRANSITION_SKILL_H_
#define LIFECYCLE_TRANSITION_SKILL_H_

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"

class LifecycleTransitionSkill final
    : public intrinsic::skills::SkillInterface {
 public:
  /**
   * @copydoc intrinsic::skills::SkillInterface:: CreateSkill
   */
  static std::unique_ptr<intrinsic::skills::SkillInterface> CreateSkill();

  /**
   * @copydoc intrinsic::skills::SkillInterface:: Preview
   */
  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> Preview(
      const intrinsic::skills::PreviewRequest& request,
      intrinsic::skills::PreviewContext& context) override;

  /**
   * @copydoc intrinsic::skills::SkillInterface:: Execute
   */
  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> Execute(
      const intrinsic::skills::ExecuteRequest& request,
      intrinsic::skills::ExecuteContext& context) override;
};

#endif  // LIFECYCLE_TRANSITION_SKILL_H_
