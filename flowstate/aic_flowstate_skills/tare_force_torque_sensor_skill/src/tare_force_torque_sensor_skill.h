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

#ifndef TARE_FORCE_TORQUE_SENSOR_SKILL_H_
#define TARE_FORCE_TORQUE_SENSOR_SKILL_H_

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"

class TareForceTorqueSensorSkill final : public intrinsic::skills::SkillInterface {
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

#endif  // TARE_FORCE_TORQUE_SENSOR_SKILL_H_
