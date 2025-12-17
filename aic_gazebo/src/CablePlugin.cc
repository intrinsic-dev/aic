/*
 * Copyright (C) 2025 Intrinsic Innovation LLC
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

#include "CablePlugin.hh"

#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/Util.hh>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

using namespace gz;
using namespace sim;

GZ_ADD_PLUGIN(aic_gazebo::CablePlugin, gz::sim::System,
              aic_gazebo::CablePlugin::ISystemConfigure,
              aic_gazebo::CablePlugin::ISystemPreUpdate,
              aic_gazebo::CablePlugin::ISystemUpdate,
              aic_gazebo::CablePlugin::ISystemPostUpdate,
              aic_gazebo::CablePlugin::ISystemReset)

namespace {
  Entity findLinkInModel(const std::string _modelName,
                  const std::string _linkName,
                  const gz::sim::EntityComponentManager &_ecm) {

    auto entitiesMatchingName = entitiesFromScopedName(
      _modelName, _ecm);

    Entity modelEntity{kNullEntity};
    if (entitiesMatchingName.size() == 1)
    {
      modelEntity = *entitiesMatchingName.begin();
    }
    if (kNullEntity != modelEntity)
    {
      return _ecm.EntityByComponents(components::Link(),
          components::ParentEntity(modelEntity),
          components::Name(_linkName));
    }
    else
    {
      gzwarn << "Model " << _modelName
              << " could not be found.\n";
    }
    return kNullEntity;
  }
}

namespace aic_gazebo {
//////////////////////////////////////////////////
void CablePlugin::Configure(
    const gz::sim::Entity& _entity,
    const std::shared_ptr<const sdf::Element>& /*_sdf*/,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager& /*_eventManager*/) {
  gzdbg << "aic_gazebo::CablePlugin::Configure on entity: " << _entity
        << std::endl;

  this->model = Model(_entity);
  if (!this->model.Valid(_ecm)) {
    gzerr << "CablePlugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // \todo(iche033) Make this configurable
  this->cableModelName = "lc_sc_cable";
  this->cableConnectionLinkName = "cable_end_0";

  this->endEffectorModelName = "ur5e";
  this->endEffectorConnectionLinkName = "ati/tool_link";

  this->connectionLocalOffset = math::Pose3d(0, 0, 0.18, 0, 0, 0);
  // this->connectionLocalOffset = math::Pose3d(0.0, -1.2, -0.5, 0, 0, 0);

  this->createJointDelay = std::chrono::seconds(8);
}

//////////////////////////////////////////////////
void CablePlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                              gz::sim::EntityComponentManager& _ecm) {

  if (this->cableLinkEntity == kNullEntity) {
    this->cableLinkEntity = findLinkInModel(
        this->cableModelName, cableConnectionLinkName, _ecm);

    if (this->cableLinkEntity == kNullEntity)
      gzerr << "Uable to find cable connection link" << std::endl;
  }

  if (this->endEffectorLinkEntity == kNullEntity) {
    this->endEffectorLinkEntity = findLinkInModel(
        this->endEffectorModelName, endEffectorConnectionLinkName, _ecm);

    if (this->endEffectorLinkEntity == kNullEntity)
      gzerr << "Unable to find end effector connection link" << std::endl;
  }

  if (this->endEffectorLinkEntity == kNullEntity &&
      this->cableLinkEntity != kNullEntity)
    return;

  if (this->detachableJointEntity == kNullEntity) {

    if (!this->moveCableToEndEffector) {
      if (_info.simTime > this->createJointDelay) {
        this->moveCableToEndEffector = true;
      }
    }

    if (this->createConnectionJoint) {
      this->detachableJointEntity = _ecm.CreateEntity();
      _ecm.CreateComponent(
          this->detachableJointEntity,
          components::DetachableJoint({this->endEffectorLinkEntity,
          //components::DetachableJoint({worldEntity(_ecm),
                                       this->cableLinkEntity, "fixed"}));
      this->createConnectionJoint = false;
    } else if (this->moveCableToEndEffector) {
      math::Pose3d endEffectorConnectionLinkWorldPose =
          gz::sim::worldPose(this->endEffectorLinkEntity, _ecm);
      auto endEffectorConnectionOffsetWorld =
        endEffectorConnectionLinkWorldPose * connectionLocalOffset;

      math::Pose3d cablePlacementPoseWorld = math::Pose3d(
          // gz::sim::worldPose(this->model.Entity(), _ecm).Pos() + math::Vector3d(0, 0.1, 1.0),
          endEffectorConnectionOffsetWorld.Pos(),
          math::Quaterniond::Identity);
      _ecm.SetComponentData<components::WorldPoseCmd>(this->model.Entity(),
          cablePlacementPoseWorld);
      this->createConnectionJoint = true;
    }
  }
}

//////////////////////////////////////////////////
void CablePlugin::Update(const gz::sim::UpdateInfo& /*_info*/,
                           gz::sim::EntityComponentManager& /*_ecm*/) {}

//////////////////////////////////////////////////
void CablePlugin::PostUpdate(
    const gz::sim::UpdateInfo& /*_info*/,
    const gz::sim::EntityComponentManager& /*_ecm*/) {}

//////////////////////////////////////////////////
void CablePlugin::Reset(const gz::sim::UpdateInfo& /*_info*/,
                          gz::sim::EntityComponentManager& /*_ecm*/) {
  gzdbg << "aic_gazebo::CablePlugin::Reset" << std::endl;
}
}  // namespace aic_gazebo
