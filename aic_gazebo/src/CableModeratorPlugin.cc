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

#include "CableModeratorPlugin.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <functional>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/CollisionBitmask.hh>
#include <gz/sim/components/CanonicalLink.hh>
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/config.hh>
#include <queue>
#include <sdf/Joint.hh>
#include <unordered_map>
#include <algorithm>

using namespace gz;
using namespace sim;

GZ_ADD_PLUGIN(aic_gazebo::CableModeratorPlugin, gz::sim::System,
              aic_gazebo::CableModeratorPlugin::ISystemConfigure,
              aic_gazebo::CableModeratorPlugin::ISystemPreUpdate,
              aic_gazebo::CableModeratorPlugin::ISystemUpdate,
              aic_gazebo::CableModeratorPlugin::ISystemPostUpdate,
              aic_gazebo::CableModeratorPlugin::ISystemReset)

namespace {

/// \brief Find link in a model
/// \param[in] _modelName Name of model
/// \param[in] _linkName Name of link to find
/// \param[in] _ecm Entity component manager
Entity findLinkInModel(const std::string& _modelName,
                       const std::string& _linkName,
                       const gz::sim::EntityComponentManager& _ecm) {
  auto entitiesMatchingName = entitiesFromScopedName(_modelName, _ecm);

  Entity modelEntity{kNullEntity};
  if (entitiesMatchingName.size() == 1) {
    modelEntity = *entitiesMatchingName.begin();
  }
  if (kNullEntity != modelEntity) {
    return _ecm.EntityByComponents(components::Link(),
                                   components::ParentEntity(modelEntity),
                                   components::Name(_linkName));
  }
  return kNullEntity;
}

constexpr uint16_t kStaticModelCategory = 2;
constexpr uint16_t kStaticModelCollideMask = 2;
constexpr uint16_t kStaticCableCategory = 1;
constexpr uint16_t kStaticCableCollideMask = 1;
constexpr uint16_t kDefaultCollideMask = 0xFFFF;

}  // namespace

namespace aic_gazebo {

// 5 mounts * 2 ports per mount = 10 SFP ports total
constexpr size_t kExpectedSfpPorts = 10;
// 5 SC ports total
constexpr size_t kExpectedScPorts = 5;

//////////////////////////////////////////////////
void CableModeratorPlugin::Configure(
    const gz::sim::Entity& /*_entity*/,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager& _eventManager) {
  gzdbg << "aic_gazebo::CableModeratorPlugin::Configure " << std::endl;

  auto cableElem = _sdf->FindElement("cable");
  while (cableElem) {
    CableConfig config;
    auto nameElem = cableElem->FindElement("name");
    if (nameElem) {
      config.modelName = nameElem->Get<std::string>();
    } else {
      config.modelName = cableElem->Get<std::string>();
    }

    if (config.modelName.empty()) {
      cableElem = cableElem->GetNextElement("cable");
      continue;
    }

    if (cableElem->HasElement("cable_connection_0_link")) {
      config.connection0LinkName =
          cableElem->Get<std::string>("cable_connection_0_link");
    } else {
      gzerr << "Missing <cable_connection_0_link> parameter." << std::endl;
      cableElem = cableElem->GetNextElement("cable");
      continue;
    }

    if (cableElem->HasElement("cable_connection_0_port")) {
      config.connection0PortName =
          cableElem->Get<std::string>("cable_connection_0_port");
    } else {
      gzerr << "Missing <cable_connection_0_port> parameter." << std::endl;
      cableElem = cableElem->GetNextElement("cable");
      continue;
    }

    if (cableElem->HasElement("cable_connection_1_link")) {
      config.connection1LinkName =
          cableElem->Get<std::string>("cable_connection_1_link");
    } else {
      gzerr << "Missing <cable_connection_1_link> parameter." << std::endl;
      cableElem = cableElem->GetNextElement("cable");
      continue;
    }

    if (cableElem->HasElement("cable_connection_1_port")) {
      config.connection1PortName =
          cableElem->Get<std::string>("cable_connection_1_port");
    } else {
      gzerr << "Missing <cable_connection_1_port> parameter." << std::endl;
      cableElem = cableElem->GetNextElement("cable");
      continue;
    }

    this->cableConfigs.push_back(config);
    cableElem = cableElem->GetNextElement("cable");
  }

  if (_sdf->HasElement("cable_mounts")) {
    auto cableMountsElem = _sdf->FindElement("cable_mounts");
    auto modelNameElem = cableMountsElem->FindElement("model_name");
    while (modelNameElem) {
      std::string modelName = modelNameElem->Get<std::string>();
      if (!modelName.empty()) {
        this->cableMounts[modelName] = false;
        gzdbg << "Added cable mount model: " << modelName << std::endl;
      }
      modelNameElem = modelNameElem->GetNextElement("model_name");
    }
  }
  if (this->cableMounts.empty()) {
    this->allCableMountsUpdated = true;
  }

  this->cableTrackers.resize(this->cableConfigs.size());

  if (_sdf->HasElement("end_effector_model")) {
    this->endEffectorModelName = _sdf->Get<std::string>("end_effector_model");
    gzdbg << "Moderator targeting end effector model: "
          << this->endEffectorModelName << std::endl;
  }
  if (_sdf->HasElement("end_effector_link")) {
    this->endEffectorLinkName = _sdf->Get<std::string>("end_effector_link");
    gzdbg << "Moderator targeting end effector link: "
          << this->endEffectorLinkName << std::endl;
  }

  this->creator = std::make_unique<SdfEntityCreator>(_ecm, _eventManager);

  this->cableInsertionPub = this->node.Advertise<gz::msgs::StringMsg>(
      "/cable_moderator/insertion_event");

  this->cableActivationPub = this->node.Advertise<gz::msgs::StringMsg>(
      "/cable_moderator/cable_activated");

  // Manual grasp subscribers for all cables
  for (size_t i = 0; i < this->cableTrackers.size(); ++i) {
    const auto& config = this->cableConfigs[i];
    auto& tracker = this->cableTrackers[i];
    std::string prefix = "/" + config.modelName;

    this->manualGraspSubs.push_back(this->node.CreateSubscriber(
        prefix + "/attach_end_0", std::function<void(const gz::msgs::Empty&)>(
                                      [&tracker](const gz::msgs::Empty&) {
                                        tracker.attachEnd0Requested = true;
                                      })));
    this->manualGraspSubs.push_back(this->node.CreateSubscriber(
        prefix + "/detach_end_0", std::function<void(const gz::msgs::Empty&)>(
                                      [&tracker](const gz::msgs::Empty&) {
                                        tracker.detachEnd0Requested = true;
                                      })));
    this->manualGraspSubs.push_back(this->node.CreateSubscriber(
        prefix + "/attach_end_1", std::function<void(const gz::msgs::Empty&)>(
                                      [&tracker](const gz::msgs::Empty&) {
                                        tracker.attachEnd1Requested = true;
                                      })));
    this->manualGraspSubs.push_back(this->node.CreateSubscriber(
        prefix + "/detach_end_1", std::function<void(const gz::msgs::Empty&)>(
                                      [&tracker](const gz::msgs::Empty&) {
                                        tracker.detachEnd1Requested = true;
                                      })));
  }
}

//////////////////////////////////////////////////
void CableModeratorPlugin::ProcessManualGraspRequests(
    gz::sim::EntityComponentManager& _ecm) {
  for (size_t i = 0; i < this->cableTrackers.size(); ++i) {
    auto& tracker = this->cableTrackers[i];
    if (!tracker.found) continue;

    if (tracker.attachEnd0Requested.exchange(false)) {
      gzdbg << "Received manual attach request for "
            << this->cableConfigs[i].modelName << " end 0" << std::endl;
      if (this->endEffectorLinkEntity == kNullEntity) {
        gzerr << "Cannot attach: End effector link not found yet." << std::endl;
        continue;
      }
      if (this->FindGripperJoint(tracker.connection0LinkEntity, _ecm) ==
          kNullEntity) {
        Entity jointEntity = _ecm.CreateEntity();
        _ecm.CreateComponent(jointEntity,
                             components::DetachableJoint(
                                 {this->endEffectorLinkEntity,
                                  tracker.connection0LinkEntity, "fixed"}));
        _ecm.CreateComponent(jointEntity,
            components::DetachableJointEnforceFixedConstraint(true));
        gzmsg << "Manually attached " << this->cableConfigs[i].modelName
              << " end 0" << std::endl;
      }
    }
    if (tracker.detachEnd0Requested.exchange(false)) {
      gzdbg << "Received manual detach request for "
            << this->cableConfigs[i].modelName << " end 0" << std::endl;
      Entity gripperJoint =
          this->FindGripperJoint(tracker.connection0LinkEntity, _ecm);
      if (gripperJoint != kNullEntity) {
        _ecm.RequestRemoveEntity(gripperJoint);
        gzmsg << "Manually detached " << this->cableConfigs[i].modelName
              << " end 0" << std::endl;
      }
    }

    if (tracker.attachEnd1Requested.exchange(false)) {
      gzdbg << "Received manual attach request for "
            << this->cableConfigs[i].modelName << " end 1" << std::endl;
      if (this->endEffectorLinkEntity == kNullEntity) {
        gzerr << "Cannot attach: End effector link not found yet." << std::endl;
        continue;
      }
      if (this->FindGripperJoint(tracker.connection1LinkEntity, _ecm) ==
          kNullEntity) {
        Entity jointEntity = _ecm.CreateEntity();
        _ecm.CreateComponent(jointEntity,
                             components::DetachableJoint(
                                 {this->endEffectorLinkEntity,
                                  tracker.connection1LinkEntity, "fixed"}));
        _ecm.CreateComponent(jointEntity,
            components::DetachableJointEnforceFixedConstraint(true));
        gzmsg << "Manually attached " << this->cableConfigs[i].modelName
              << " end 1" << std::endl;
      }
    }
    if (tracker.detachEnd1Requested.exchange(false)) {
      gzdbg << "Received manual detach request for "
            << this->cableConfigs[i].modelName << " end 1" << std::endl;
      Entity gripperJoint =
          this->FindGripperJoint(tracker.connection1LinkEntity, _ecm);
      if (gripperJoint != kNullEntity) {
        _ecm.RequestRemoveEntity(gripperJoint);
        gzmsg << "Manually detached " << this->cableConfigs[i].modelName
              << " end 1" << std::endl;
      }
    }
  }
}

//////////////////////////////////////////////////
void CableModeratorPlugin::MakeCableStatic(
    size_t _cableIndex, gz::sim::EntityComponentManager& _ecm) {
  auto& tracker = this->cableTrackers[_cableIndex];
  tracker.isDynamic = false;
#if (GZ_SIM_MAJOR_VERSION == 9 && GZ_SIM_MINOR_VERSION >= 6) ||  \
    (GZ_SIM_MAJOR_VERSION == 10 && GZ_SIM_MINOR_VERSION >= 3) || \
    (GZ_SIM_MAJOR_VERSION > 10)
  Model(tracker.modelEntity).SetStatic(_ecm, true);
  math::Pose3d pose = worldPose(tracker.modelEntity, _ecm);
  gzmsg << "Made cable [" << this->cableConfigs[_cableIndex].modelName
        << "] static at pose: " << pose << std::endl;
#else
  static bool warnedOnce = false;
  if (!warnedOnce) {
    gzwarn << "Unable to set model static state in version "
           << GZ_SIM_VERSION_FULL << std::endl;
    warnedOnce = true;
  }
#endif

  if (tracker.detachableJointStatic0Entity != kNullEntity) {
    _ecm.RequestRemoveEntity(tracker.detachableJointStatic0Entity);
    tracker.detachableJointStatic0Entity = kNullEntity;
  }
  if (tracker.detachableJointStatic1Entity != kNullEntity) {
    _ecm.RequestRemoveEntity(tracker.detachableJointStatic1Entity);
    tracker.detachableJointStatic1Entity = kNullEntity;
  }
}

//////////////////////////////////////////////////
void CableModeratorPlugin::MakeCableDynamic(
    size_t _cableIndex, gz::sim::EntityComponentManager& _ecm) {
  auto& tracker = this->cableTrackers[_cableIndex];
  tracker.isDynamic = true;
#if (GZ_SIM_MAJOR_VERSION == 9 && GZ_SIM_MINOR_VERSION >= 6) ||  \
    (GZ_SIM_MAJOR_VERSION == 10 && GZ_SIM_MINOR_VERSION >= 3) || \
    (GZ_SIM_MAJOR_VERSION > 10)
  Model(tracker.modelEntity).SetStatic(_ecm, false);
  gz::msgs::StringMsg pubMsg;
  pubMsg.set_data(this->cableConfigs[_cableIndex].modelName);
  this->cableActivationPub.Publish(pubMsg);
#else
  static bool warnedOnce = false;
  if (!warnedOnce) {
    gzwarn << "Unable to set model static state in version "
           << GZ_SIM_VERSION_FULL << std::endl;
    warnedOnce = true;
  }
  (void)_ecm;
#endif
}

//////////////////////////////////////////////////
void CableModeratorPlugin::FindCableMounts(
    gz::sim::EntityComponentManager& _ecm) {
  bool allDone = true;
  for (auto& item : this->cableMounts) {
    if (item.second) continue;

    auto entitiesMatchingName = entitiesFromScopedName(item.first, _ecm);
    if (!entitiesMatchingName.empty()) {
      Entity modelEntity = *entitiesMatchingName.begin();
#if (GZ_SIM_MAJOR_VERSION == 9 && GZ_SIM_MINOR_VERSION >= 6) ||  \
    (GZ_SIM_MAJOR_VERSION == 10 && GZ_SIM_MINOR_VERSION >= 3) || \
    (GZ_SIM_MAJOR_VERSION > 10)
      Model(modelEntity).SetStatic(_ecm, true);
      item.second = true;
      math::Pose3d pose = worldPose(modelEntity, _ecm);
      gzmsg << "Made model [" << item.first << "] static at pose: " << pose
            << std::endl;
#else
      static bool warnedOnce = false;
      if (!warnedOnce) {
        gzwarn << "Unable to set model static state in version "
               << GZ_SIM_VERSION_FULL << std::endl;
        warnedOnce = true;
      }
      item.second = true;
#endif
      this->SetModelCollisionsBitmasks(modelEntity, kStaticModelCategory,
                                       kStaticModelCollideMask, _ecm);
    } else {
      allDone = false;
    }
  }
  this->allCableMountsUpdated = allDone;
}

//////////////////////////////////////////////////
void CableModeratorPlugin::Cleanup(gz::sim::EntityComponentManager& _ecm) {
  for (const auto& ent : this->staticEntities) {
    if (ent != kNullEntity) _ecm.RequestRemoveEntity(ent);
  }
  this->staticEntities.clear();
}

//////////////////////////////////////////////////
void CableModeratorPlugin::PreUpdate(const gz::sim::UpdateInfo& /*_info*/,
                                     gz::sim::EntityComponentManager& _ecm) {
  if (!this->allCableMountsUpdated) {
    this->FindCableMounts(_ecm);
  }

  this->FindCableModels(_ecm);
  this->FindPortEntities(_ecm);

  this->ProcessManualGraspRequests(_ecm);

  if (this->endEffectorLinkEntity == kNullEntity) {
    this->endEffectorLinkEntity = findLinkInModel(
        this->endEffectorModelName, this->endEffectorLinkName, _ecm);
    if (this->endEffectorLinkEntity != kNullEntity) {
      gzmsg << "End effector link discovered: " << this->endEffectorLinkName
            << std::endl;
    }
  }
  if (this->endEffectorLinkEntity == kNullEntity) return;

  for (size_t i = 0; i < this->cableTrackers.size(); ++i) {
    auto& tracker = this->cableTrackers[i];
    if (!tracker.found || tracker.isCompleted) continue;

    // Helper lambda: check if a contact involves a gripper/finger link
    auto hasGripperContact =
        [&](Entity collisionEnt,
            const std::unordered_set<Entity>& ownCollisions) -> bool {
      auto contactComp =
          _ecm.Component<components::ContactSensorData>(collisionEnt);
      if (!contactComp) return false;
      for (const auto& contact : contactComp->Data().contact()) {
        // Determine which collision in the pair is the "other" (not ours)
        Entity otherCollision = kNullEntity;
        Entity c1 = contact.collision1().id();
        Entity c2 = contact.collision2().id();
        if (ownCollisions.count(c1) > 0) {
          otherCollision = c2;
        } else if (ownCollisions.count(c2) > 0) {
          otherCollision = c1;
        } else {
          continue;
        }
        // Walk up to the parent link of the other collision
        auto parentComp =
            _ecm.Component<components::ParentEntity>(otherCollision);
        if (!parentComp) continue;
        auto nameComp = _ecm.Component<components::Name>(parentComp->Data());
        if (!nameComp) continue;
        const std::string& linkName = nameComp->Data();
        if (linkName.find(this->endEffectorLinkName) != std::string::npos) {
          return true;
        }
      }
      return false;
    };

    bool end0InContact = false;
    bool end1InContact = false;
    int currentContactEnd = -1;

    for (Entity collisionEnt : tracker.end0CollisionEntities) {
      if (hasGripperContact(collisionEnt, tracker.end0CollisionEntities)) {
        end0InContact = true;
        break;
      }
    }

    for (Entity collisionEnt : tracker.end1CollisionEntities) {
      if (hasGripperContact(collisionEnt, tracker.end1CollisionEntities)) {
        end1InContact = true;
        break;
      }
    }

    if (end0InContact) {
      currentContactEnd = 0;
    } else if (end1InContact) {
      currentContactEnd = 1;
    }

    if (currentContactEnd != -1) {
      // Create subscribers if they weren't already.
      if (tracker.portSubs.empty()) this->CreatePortSubscribers(i);

      int prevContactEnd = tracker.activeContactEnd.load();
      if (prevContactEnd == -1 || prevContactEnd != currentContactEnd) {
        gzmsg << "Cable " << this->cableConfigs[i].modelName
              << " contact confirmed near End " << currentContactEnd
              << std::endl;
        tracker.activeContactEnd.store(currentContactEnd);
        if (prevContactEnd == -1) {
          this->MakeCableDynamic(i, _ecm);
          this->SetModelCollisionsBitmasks(tracker.modelEntity,
                                           kDefaultCollideMask,
                                           kDefaultCollideMask, _ecm);

        }
      }

      if (currentContactEnd == 0) {
        // Grasping near End 0: Unfreeze End 0 (if not inserted), Ensure End 1
        // is frozen
        if (!tracker.end0Inserted &&
            tracker.detachableJointStatic0Entity != kNullEntity) {
          _ecm.RequestRemoveEntity(tracker.detachableJointStatic0Entity);
          tracker.detachableJointStatic0Entity = kNullEntity;
          gzmsg << "Unfreezing End 0 for manipulation." << std::endl;
        }
        if (!tracker.end1Inserted &&
            tracker.detachableJointStatic1Entity == kNullEntity) {
          tracker.detachableJointStatic1Entity =
              this->MakeStatic(tracker.connection1LinkEntity, true, _ecm);
          gzdbg << "Locking End 1. Resulting joint: "
                << tracker.detachableJointStatic1Entity << std::endl;
        }
      } else {
        // Grasping near End 1: Unfreeze End 1 (if not inserted), Ensure End 0
        // is frozen
        if (!tracker.end1Inserted &&
            tracker.detachableJointStatic1Entity != kNullEntity) {
          _ecm.RequestRemoveEntity(tracker.detachableJointStatic1Entity);
          tracker.detachableJointStatic1Entity = kNullEntity;
          gzmsg << "Unfreezing End 1 for manipulation." << std::endl;
        }
        if (!tracker.end0Inserted &&
            tracker.detachableJointStatic0Entity == kNullEntity) {
          tracker.detachableJointStatic0Entity =
              this->MakeStatic(tracker.connection0LinkEntity, true, _ecm);
          gzdbg << "Locking End 0. Resulting joint: "
                << tracker.detachableJointStatic0Entity << std::endl;
        }
      }
    } else {
      // Dynamic State Persistence: once made dynamic, keep it dynamic even
      // without active contact.
      tracker.activeContactEnd.store(-1);
    }

    if (tracker.isDynamic) {
      Entity graspJoint = this->FindExternalGraspJoint(tracker, _ecm);
      if (graspJoint != kNullEntity) {
        auto closerEnd = this->FindGraspedEnd(i, graspJoint, _ecm);
        if (closerEnd.has_value()) {
          tracker.lastGraspedEnd = closerEnd;
        }
        tracker.activeGraspJoint.store(graspJoint);
      } else {
        tracker.activeGraspJoint.store(kNullEntity);
        tracker.lastGraspedEnd = std::nullopt;
      }
    }

    // Distance-based insertion check
    if (tracker.isDynamic) {
      Entity graspJoint = tracker.activeGraspJoint.load();
      if (graspJoint != kNullEntity) {
        int graspedEnd = tracker.lastGraspedEnd.value_or(-1);
        if (graspedEnd == 0 && !tracker.end0Inserted) {
          if (this->CheckDistanceInsertion(i, 0, _ecm)) {
            tracker.end0Inserted = true;
            gzmsg << "Distance-based insertion detected for " << this->cableConfigs[i].modelName << " End 0" << std::endl;
            this->PublishInsertionEvent(i, 0, _ecm);
          }
        } else if (graspedEnd == 1 && !tracker.end1Inserted) {
          if (this->CheckDistanceInsertion(i, 1, _ecm)) {
            tracker.end1Inserted = true;
            gzmsg << "Distance-based insertion detected for " << this->cableConfigs[i].modelName << " End 1" << std::endl;
            this->PublishInsertionEvent(i, 1, _ecm);
          }
        }
      }
    }

    // Make a cable end static once it is inserted
    if (tracker.end0Inserted &&
        tracker.detachableJointStatic0Entity == kNullEntity) {
      tracker.detachableJointStatic0Entity =
          this->MakeStatic(tracker.connection0LinkEntity, true, _ecm);
      this->DisableLinkCollisions(tracker.connection0LinkEntity, _ecm);
      gzdbg << "Locking End 0 after insertion. Resulting joint: "
            << tracker.detachableJointStatic0Entity << std::endl;

    }
    if (tracker.end1Inserted &&
        tracker.detachableJointStatic1Entity == kNullEntity) {
      tracker.detachableJointStatic1Entity =
          this->MakeStatic(tracker.connection1LinkEntity, true, _ecm);
      this->DisableLinkCollisions(tracker.connection1LinkEntity, _ecm);
      gzdbg << "Locking End 1 after insertion. Resulting joint: "
            << tracker.detachableJointStatic1Entity << std::endl;
    }

    if (tracker.end0Inserted && tracker.end1Inserted && !tracker.isCompleted) {
      tracker.isCompleted = true;
      this->MakeCableStatic(i, _ecm);
      gzmsg << "Cable " << this->cableConfigs[i].modelName << " fully inserted."
            << std::endl;
      tracker.portSubs.clear();
    }
  }
}

//////////////////////////////////////////////////
void CableModeratorPlugin::Update(const gz::sim::UpdateInfo&,
                                  gz::sim::EntityComponentManager&) {}
void CableModeratorPlugin::PostUpdate(const gz::sim::UpdateInfo&,
                                      const gz::sim::EntityComponentManager&) {}
void CableModeratorPlugin::Reset(const gz::sim::UpdateInfo&,
                                 gz::sim::EntityComponentManager&) {
  gzdbg << "aic_gazebo::CableModeratorPlugin::Reset" << std::endl;
}

//////////////////////////////////////////////////
Entity CableModeratorPlugin::MakeStatic(Entity _entity,
                                        bool _attachEntityAsParentOfJoint,
                                        EntityComponentManager& _ecm) {
  Entity detachableJointEntity = kNullEntity;

  static sdf::Model staticModelToSpawn;
  if (staticModelToSpawn.LinkCount() == 0u) {
    sdf::ElementPtr staticModelSDF(new sdf::Element);
    sdf::initFile("model.sdf", staticModelSDF);
    staticModelSDF->GetAttribute("name")->Set("static_model");
    staticModelSDF->GetElement("static")->Set(true);
    sdf::ElementPtr linkElem = staticModelSDF->AddElement("link");
    linkElem->GetAttribute("name")->Set("static_link");
    staticModelToSpawn.Load(staticModelSDF);
  }

  auto parentComp = _ecm.Component<components::ParentEntity>(_entity);
  auto parentNameComp = _ecm.Component<components::Name>(parentComp->Data());
  auto nameComp = _ecm.Component<components::Name>(_entity);
  std::string staticEntName =
      nameComp->Data() + "_" + parentNameComp->Data() + "__static__";
  Entity staticEntity =
      _ecm.EntityByComponents(components::Name(staticEntName));
  if (staticEntity == kNullEntity) {
    staticModelToSpawn.SetName(staticEntName);
    staticEntity = this->creator->CreateEntities(&staticModelToSpawn);
    this->staticEntities.insert(staticEntity);
    this->creator->SetParent(staticEntity,
                             _ecm.EntityByComponents(components::World()));
  }

  math::Pose3d worldPoseOfLink = worldPose(_entity, _ecm);
  _ecm.SetComponentData<components::Pose>(staticEntity, worldPoseOfLink);

  Entity staticLinkEntity = _ecm.EntityByComponents(
      components::Link(), components::ParentEntity(staticEntity),
      components::Name("static_link"));

  if (staticLinkEntity == kNullEntity) return detachableJointEntity;

  Entity parentLinkEntity;
  Entity childLinkEntity;
  // TODO(anyone) This function is never called with this argument set as false
  // Check if we need it or we can remove it.
  if (_attachEntityAsParentOfJoint) {
    parentLinkEntity = _entity;
    childLinkEntity = staticLinkEntity;
  } else {
    parentLinkEntity = staticLinkEntity;
    childLinkEntity = _entity;
  }

  detachableJointEntity = _ecm.CreateEntity();
  _ecm.CreateComponent(detachableJointEntity,
                       components::DetachableJoint(
                           {parentLinkEntity, childLinkEntity, "fixed"}));
  _ecm.CreateComponent(detachableJointEntity,
      components::DetachableJointEnforceFixedConstraint(false));

  return detachableJointEntity;
}

//////////////////////////////////////////////////
Entity CableModeratorPlugin::FindGripperJoint(
    Entity _connectionLinkEntity, const EntityComponentManager& _ecm) const {
  Entity result = kNullEntity;
  _ecm.Each<components::DetachableJoint>(
      [&](const Entity& _entity,
          const components::DetachableJoint* _joint) -> bool {
        const auto& info = _joint->Data();
        if ((info.parentLink == this->endEffectorLinkEntity &&
             info.childLink == _connectionLinkEntity) ||
            (info.parentLink == _connectionLinkEntity &&
             info.childLink == this->endEffectorLinkEntity)) {
          result = _entity;
          return false;
        }
        return true;
      });
  return result;
}

//////////////////////////////////////////////////
Entity CableModeratorPlugin::FindExternalGraspJoint(
    const CableTracker& _tracker,
    const gz::sim::EntityComponentManager& _ecm) const {
  Entity result = kNullEntity;
  _ecm.Each<components::DetachableJoint>(
      [&](const Entity& _entity,
          const components::DetachableJoint* _joint) -> bool {
        const auto& info = _joint->Data();
        bool parentInCable = _tracker.neighbors.count(info.parentLink) > 0;
        bool childInCable = _tracker.neighbors.count(info.childLink) > 0;
        if (parentInCable != childInCable) {
          Entity externalLink =
              parentInCable ? info.childLink : info.parentLink;
          auto parentEnt =
              _ecm.Component<components::ParentEntity>(externalLink);
          if (parentEnt) {
            auto modelName =
                _ecm.Component<components::Name>(parentEnt->Data());
            if (modelName &&
                modelName->Data().find("__static__") != std::string::npos) {
              return true;
            }
          }
          result = _entity;
          return false;
        }
        return true;
      });
  return result;
}

//////////////////////////////////////////////////
void CableModeratorPlugin::FindCableModels(EntityComponentManager& _ecm) {
  for (size_t i = 0; i < this->cableConfigs.size(); ++i) {
    if (!this->cableTrackers[i].found) {
      auto entitiesMatchingName =
          entitiesFromScopedName(this->cableConfigs[i].modelName, _ecm);
      if (!entitiesMatchingName.empty()) {
        Entity modelEntity = *entitiesMatchingName.begin();
        this->cableTrackers[i].modelEntity = modelEntity;
        this->cableTrackers[i].found = true;
        Model model(modelEntity);
        this->cableTrackers[i].connection0LinkEntity =
            model.LinkByName(_ecm, this->cableConfigs[i].connection0LinkName);
        this->cableTrackers[i].connection1LinkEntity =
            model.LinkByName(_ecm, this->cableConfigs[i].connection1LinkName);

        // Build the topological graph of the cable
        for (Entity jointEnt : model.Joints(_ecm)) {
          auto pName = _ecm.Component<components::ParentLinkName>(jointEnt);
          auto cName = _ecm.Component<components::ChildLinkName>(jointEnt);
          if (pName && cName) {
            Entity pEnt = model.LinkByName(_ecm, pName->Data());
            Entity cEnt = model.LinkByName(_ecm, cName->Data());
            if (pEnt != kNullEntity && cEnt != kNullEntity) {
              this->cableTrackers[i].neighbors[pEnt].push_back(cEnt);
              this->cableTrackers[i].neighbors[cEnt].push_back(pEnt);
            }
          }
        }

        // Initialize monitored links (connection links and their immediate
        // fixed neighbors)
        auto& tracker = this->cableTrackers[i];
        tracker.end0MonitoredLinks.insert(tracker.connection0LinkEntity);
        tracker.end1MonitoredLinks.insert(tracker.connection1LinkEntity);

        for (Entity jointEnt : model.Joints(_ecm)) {
          auto jointTypeComp = _ecm.Component<components::JointType>(jointEnt);
          if (jointTypeComp && jointTypeComp->Data() == sdf::JointType::FIXED) {
            auto pName = _ecm.Component<components::ParentLinkName>(jointEnt);
            auto cName = _ecm.Component<components::ChildLinkName>(jointEnt);
            if (pName && cName) {
              Entity pEnt = model.LinkByName(_ecm, pName->Data());
              Entity cEnt = model.LinkByName(_ecm, cName->Data());
              if (pEnt != kNullEntity && cEnt != kNullEntity) {
                if (pEnt == tracker.connection0LinkEntity) {
                  tracker.end0MonitoredLinks.insert(cEnt);
                } else if (cEnt == tracker.connection0LinkEntity) {
                  tracker.end0MonitoredLinks.insert(pEnt);
                }

                if (pEnt == tracker.connection1LinkEntity) {
                  tracker.end1MonitoredLinks.insert(cEnt);
                } else if (cEnt == tracker.connection1LinkEntity) {
                  tracker.end1MonitoredLinks.insert(pEnt);
                }
              }
            }
          }
        }

        // Discover and register collision entities for monitored links
        _ecm.Each<components::Collision, components::ParentEntity>(
            [&](const Entity& _collisionEntity, const components::Collision*,
                const components::ParentEntity* _parentLinkComp) -> bool {
              Entity linkEntity = _parentLinkComp->Data();
              if (tracker.end0MonitoredLinks.count(linkEntity) > 0) {
                tracker.end0CollisionEntities.insert(_collisionEntity);
                if (!_ecm.EntityHasComponentType(
                        _collisionEntity,
                        components::ContactSensorData::typeId)) {
                  _ecm.CreateComponent(_collisionEntity,
                                       components::ContactSensorData());
                  std::string collisionName = std::to_string(_collisionEntity);
                  if (auto nameComp =
                          _ecm.Component<components::Name>(_collisionEntity)) {
                    collisionName = nameComp->Data();
                  }
                  std::string parentName = std::to_string(linkEntity);
                  if (auto parentNameComp =
                          _ecm.Component<components::Name>(linkEntity)) {
                    parentName = parentNameComp->Data();
                  }
                  gzdbg << "Enabled contact tracking on End 0 collision: "
                        << collisionName << " (parent: " << parentName << ")"
                        << std::endl;
                }
              } else if (tracker.end1MonitoredLinks.count(linkEntity) > 0) {
                tracker.end1CollisionEntities.insert(_collisionEntity);
                if (!_ecm.EntityHasComponentType(
                        _collisionEntity,
                        components::ContactSensorData::typeId)) {
                  _ecm.CreateComponent(_collisionEntity,
                                       components::ContactSensorData());
                  std::string collisionName = std::to_string(_collisionEntity);
                  if (auto nameComp =
                          _ecm.Component<components::Name>(_collisionEntity)) {
                    collisionName = nameComp->Data();
                  }
                  std::string parentName = std::to_string(linkEntity);
                  if (auto parentNameComp =
                          _ecm.Component<components::Name>(linkEntity)) {
                    parentName = parentNameComp->Data();
                  }
                  gzdbg << "Enabled contact tracking on End 1 collision: "
                        << collisionName << " (parent: " << parentName << ")"
                        << std::endl;
                }
              }
              return true;
            });

        // Make Cable static and set masks so the plugs do not collide with
        // mount but should collide with gripper
        this->MakeCableStatic(i, _ecm);
        this->SetModelCollisionsBitmasks(tracker.modelEntity,
                                         kStaticCableCategory,
                                         kStaticCableCollideMask, _ecm);

        gzmsg << "Found cable model: " << this->cableConfigs[i].modelName
              << std::endl;
      }
    }
  }
}

//////////////////////////////////////////////////
void CableModeratorPlugin::CreatePortSubscribers(size_t _cableIndex) {
  auto& config = this->cableConfigs[_cableIndex];
  auto& tracker = this->cableTrackers[_cableIndex];

  std::vector<std::string> allTopics;
  this->node.TopicList(allTopics);

  auto cb = [this, _cableIndex](const gz::msgs::Boolean& _msg,
                                const gz::transport::MessageInfo& _info,
                                int end) {
    if (_msg.data()) {
      auto& tracker = this->cableTrackers[_cableIndex];
      // Grasp filter: Only report insertion if this cable is actually being
      // held and the touch plugin reported touch on the grasped end
      if (tracker.activeGraspJoint.load() == kNullEntity ||
          tracker.lastGraspedEnd != end) {
        gzdbg << "Ignoring touch event for "
              << this->cableConfigs[_cableIndex].modelName << " on port "
              << _info.Topic() << " (Not grasped)" << std::endl;
        return;
      }

      if (end == 0 ? tracker.end0Inserted : tracker.end1Inserted) return;
      if (end == 0)
        tracker.end0Inserted = true;
      else
        tracker.end1Inserted = true;
      gzmsg << "Touch event received for "
            << this->cableConfigs[_cableIndex].modelName << " end " << end
            << " from topic " << _info.Topic() << std::endl;
      auto pos = _info.Topic().rfind("/");
      if (pos != std::string::npos)
        tracker.touchEventCallbackNamespace = _info.Topic().substr(0, pos);
      gz::msgs::StringMsg pubMsg;
      pubMsg.set_data(this->cableConfigs[_cableIndex].modelName + "#" +
                      std::to_string(end) + "#" +
                      tracker.touchEventCallbackNamespace);
      this->cableInsertionPub.Publish(pubMsg);
    }
  };

  auto subscribe = [this, &config, &tracker, cb](const std::string& _topic,
                                                 int _end) {
    gzmsg << "Subscribing End " << _end << " of " << config.modelName
          << " to: " << _topic << std::endl;
    tracker.portSubs.emplace_back(this->node.CreateSubscriber(
        _topic, std::function<void(const gz::msgs::Boolean&,
                                   const gz::transport::MessageInfo&)>(
                    std::bind(cb, std::placeholders::_1, std::placeholders::_2,
                              _end))));
  };

  for (const auto& topic : allTopics) {
    if (topic.find("touched") == std::string::npos) continue;

    if (topic.find(config.connection0PortName) != std::string::npos) {
      subscribe(topic, 0);
    }
    if (topic.find(config.connection1PortName) != std::string::npos) {
      subscribe(topic, 1);
    }
  }
}

//////////////////////////////////////////////////
std::optional<int> CableModeratorPlugin::FindGraspedEnd(
    size_t _cableIndex, Entity _graspJoint,
    const EntityComponentManager& _ecm) const {
  auto& tracker = this->cableTrackers[_cableIndex];
  auto jointComp = _ecm.Component<components::DetachableJoint>(_graspJoint);
  if (!jointComp) return std::nullopt;
  const auto& info = jointComp->Data();

  Entity cableLink = tracker.neighbors.count(info.parentLink) > 0
                         ? info.parentLink
                         : info.childLink;

  if (cableLink == tracker.connection0LinkEntity) return 0;
  if (cableLink == tracker.connection1LinkEntity) return 1;

  // Fallback: Topological distance (link count).
  // This is used if the gripper attaches to a link that isn't one of the ends.
  // Perform BFS to find the shortest topological path to both ends using cached
  // graph
  std::unordered_map<Entity, int> dists;
  std::queue<Entity> q;
  q.push(cableLink);
  dists[cableLink] = 0;

  while (!q.empty()) {
    Entity curr = q.front();
    q.pop();
    int d = dists[curr];

    // Check if we've reached either end link
    if (curr == tracker.connection0LinkEntity) return 0;
    if (curr == tracker.connection1LinkEntity) return 1;

    auto it = tracker.neighbors.find(curr);
    if (it != tracker.neighbors.end()) {
      for (Entity next : it->second) {
        if (dists.find(next) == dists.end()) {
          dists[next] = d + 1;
          q.push(next);
        }
      }
    }
  }

  // Failed to find a connection to any link
  return std::nullopt;
}

void CableModeratorPlugin::DisableLinkCollisions(
    gz::sim::Entity _linkEntity,
    gz::sim::EntityComponentManager &_ecm) {
  this->SetLinkCollisionsBitmasks(_linkEntity, 0, 0, _ecm);
}

void CableModeratorPlugin::SetLinkCollisionsBitmasks(
    gz::sim::Entity _linkEntity,
    std::optional<uint16_t> _categoryMask,
    std::optional<uint16_t> _collideMask,
    gz::sim::EntityComponentManager &_ecm) {
  gz::sim::Link link(_linkEntity);
  for (const auto& collisionEntity : link.Collisions(_ecm)) {
    if (_categoryMask.has_value()) {
      _ecm.SetComponentData<gz::sim::components::CategoryBitmaskCmd>(
          collisionEntity, _categoryMask.value());
    }
    if (_collideMask.has_value()) {
      _ecm.SetComponentData<gz::sim::components::CollideBitmaskCmd>(
          collisionEntity, _collideMask.value());
    }
  }
}

void CableModeratorPlugin::SetModelCollisionsBitmasks(
    gz::sim::Entity _modelEntity,
    std::optional<uint16_t> _categoryMask,
    std::optional<uint16_t> _collideMask,
    gz::sim::EntityComponentManager &_ecm) {
  // Find the cable that we are trying to set collisions for
  const CableTracker* targetTracker = nullptr;
  for (const auto& tracker : this->cableTrackers) {
    if (tracker.modelEntity == _modelEntity) {
      targetTracker = &tracker;
      break;
    }
  }

  gz::sim::Model model(_modelEntity);
  for (const auto& linkEntity : model.Links(_ecm)) {
    // Skip setting this bitmask if it is for an inserted port, since  this
    // is already made static on insertion and we don't want to make it
    // dynamic again upon grasping the second end.
    if (targetTracker) {
      if (linkEntity == targetTracker->connection0LinkEntity &&
          targetTracker->end0Inserted) {
        continue;
      }
      if (linkEntity == targetTracker->connection1LinkEntity &&
          targetTracker->end1Inserted) {
        continue;
      }
    }
    gz::sim::Link link(linkEntity);
    this->SetLinkCollisionsBitmasks(linkEntity, _categoryMask, _collideMask,
                                    _ecm);
  }
}

//////////////////////////////////////////////////
void CableModeratorPlugin::FindPortEntities(EntityComponentManager& _ecm) {
  bool needSearch = false;
  for (const auto& tracker : this->cableTrackers) {
    if (tracker.found && !tracker.isCompleted) {
      if (tracker.port0LinkEntities.size() < kExpectedSfpPorts ||
          tracker.port1LinkEntities.size() < kExpectedScPorts) {
        needSearch = true;
        break;
      }
    }
  }
  if (!needSearch) return;

  _ecm.Each<components::Link, components::ParentEntity, components::Name>(
      [&](const Entity& _entity, const components::Link*,
          const components::ParentEntity* _parentPart,
          const components::Name* _nameComp) -> bool {

        Entity modelEntity = _parentPart->Data();
        auto modelNameComp = _ecm.Component<components::Name>(modelEntity);
        if (!modelNameComp) return true;

        const std::string& modelName = modelNameComp->Data();
        const std::string& linkName = _nameComp->Data();

        if (linkName.find("port") == std::string::npos) return true;
        if (linkName.find("entrance") != std::string::npos) return true;

        for (size_t i = 0; i < this->cableConfigs.size(); ++i) {
          const auto& config = this->cableConfigs[i];
          auto& tracker = this->cableTrackers[i];

          if (modelName.find(config.connection0PortName) != std::string::npos ||
              linkName.find(config.connection0PortName) != std::string::npos) {
            if (std::find(tracker.port0LinkEntities.begin(), tracker.port0LinkEntities.end(), _entity) == tracker.port0LinkEntities.end()) {
              tracker.port0LinkEntities.push_back(_entity);
              gzdbg << "Cable " << config.modelName << " End 0 matched port: "
                    << modelName << "/" << linkName
                    << " (" << tracker.port0LinkEntities.size() << "/" << kExpectedSfpPorts << ")" << std::endl;
            }
          }

          if (modelName.find(config.connection1PortName) != std::string::npos ||
              linkName.find(config.connection1PortName) != std::string::npos) {
            if (std::find(tracker.port1LinkEntities.begin(), tracker.port1LinkEntities.end(), _entity) == tracker.port1LinkEntities.end()) {
              tracker.port1LinkEntities.push_back(_entity);
              gzdbg << "Cable " << config.modelName << " End 1 matched port: "
                    << modelName << "/" << linkName
                    << " (" << tracker.port1LinkEntities.size() << "/" << kExpectedScPorts << ")" << std::endl;
            }
          }
        }
        return true;
      });
}

//////////////////////////////////////////////////
bool CableModeratorPlugin::CheckDistanceInsertion(
    size_t _cableIndex, int _end, const EntityComponentManager& _ecm) {

  auto& tracker = this->cableTrackers[_cableIndex];
  
  std::string tipLinkName = (_end == 0) ? "sfp_tip_link" : "sc_tip_link";
  Entity tipEntity = findLinkInModel(this->cableConfigs[_cableIndex].modelName, tipLinkName, _ecm);
  
  const auto& portEntities = (_end == 0) ? tracker.port0LinkEntities : tracker.port1LinkEntities;

  if (tipEntity == kNullEntity || portEntities.empty()) return false;

  math::Pose3d plugPose = gz::sim::worldPose(tipEntity, _ecm);
  const double kInsertionThreshold = 0.001; // 1mm

  for (Entity portEntity : portEntities) {
    math::Pose3d portPose = gz::sim::worldPose(portEntity, _ecm);
    double dist = plugPose.Pos().Distance(portPose.Pos());

    if (dist < kInsertionThreshold) {
      auto portNameComp = _ecm.Component<components::Name>(portEntity);
      auto parentEnt = _ecm.Component<components::ParentEntity>(portEntity);
      if (portNameComp && parentEnt) {
        auto modelNameComp = _ecm.Component<components::Name>(parentEnt->Data());
        if (modelNameComp) {
          std::string portModel = modelNameComp->Data();
          std::string portLink = portNameComp->Data();
          size_t pos = portLink.rfind("_link");
          if (pos != std::string::npos) {
            portLink = portLink.substr(0, pos);
          }
          tracker.touchEventCallbackNamespace = portModel + "/" + portLink;
        }
      }
      return true;
    }
  }
  return false;
}

//////////////////////////////////////////////////
void CableModeratorPlugin::PublishInsertionEvent(
    size_t _cableIndex, int _end, const EntityComponentManager& /*_ecm*/) {
  auto& tracker = this->cableTrackers[_cableIndex];
  gz::msgs::StringMsg pubMsg;
  pubMsg.set_data(this->cableConfigs[_cableIndex].modelName + "#" +
                  std::to_string(_end) + "#" +
                  tracker.touchEventCallbackNamespace);
  this->cableInsertionPub.Publish(pubMsg);
}

}  // namespace aic_gazebo
