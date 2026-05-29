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

#ifndef FLOWSTATE_AIC_SKILLS_LIFECYCLE_TRANSITION_SKILL_H_
#define FLOWSTATE_AIC_SKILLS_LIFECYCLE_TRANSITION_SKILL_H_

#include "intrinsic/skills/cc/skill_interface.h"

class LifecycleTransitionSkill : public intrinsic::skills::SkillInterface {
 public:
  LifecycleTransitionSkill() = default;
  ~LifecycleTransitionSkill() override = default;

  static std::unique_ptr<intrinsic::skills::SkillInterface> CreateSkill();

  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> Preview(
      const intrinsic::skills::PreviewRequest& request,
      intrinsic::skills::PreviewContext& context) override;

  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> Execute(
      const intrinsic::skills::ExecuteRequest& request,
      intrinsic::skills::ExecuteContext& context) override;
};

#endif  // FLOWSTATE_AIC_SKILLS_LIFECYCLE_TRANSITION_SKILL_H_
