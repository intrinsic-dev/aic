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
#include <gz/sim/components/CanonicalLink.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>

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
  } else {
    // gzwarn << "Model " << _modelName << " could not be found.\n";
  }
  return kNullEntity;
}

}  // namespace

namespace aic_gazebo {

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

  this->cableTrackers.resize(this->cableConfigs.size());

  if (this->cableConfigs.empty()) {
    gzerr << "Missing valid <cable> parameters." << std::endl;
    return;
  }

  if (_sdf->HasElement("end_effector_model")) {
    this->endEffectorModelName = _sdf->Get<std::string>("end_effector_model");
  } else {
    gzerr << "Missing <end_effector_model> parameter." << std::endl;
    return;
  }

  if (_sdf->HasElement("end_effector_link")) {
    this->endEffectorLinkName = _sdf->Get<std::string>("end_effector_link");
  } else {
    gzerr << "Missing <end_effector_link> parameter." << std::endl;
    return;
  }

  this->creator = std::make_unique<SdfEntityCreator>(_ecm, _eventManager);

  this->taskCompletionPub = this->node.Advertise<gz::msgs::StringMsg>(
      "/cable_moderator/insertion_event");

  gzmsg << "Initializing to NEXT_CABLE state." << std::endl;
  this->cableState = CableState::NEXT_CABLE;
}

//////////////////////////////////////////////////
void CableModeratorPlugin::ProcessManualGraspRequests(
    gz::sim::EntityComponentManager& _ecm) {
  if (this->attachEnd0Requested.exchange(false)) {
    if (this->FindGripperJoint(this->cableConnection0LinkEntity, _ecm) ==
        kNullEntity) {
      Entity jointEntity = _ecm.CreateEntity();
      _ecm.CreateComponent(jointEntity,
                           components::DetachableJoint(
                               {this->endEffectorLinkEntity,
                                this->cableConnection0LinkEntity, "fixed"}));
      gzdbg << "Manually attached end 0" << std::endl;
    }
  }
  if (this->detachEnd0Requested.exchange(false)) {
    Entity gripperJoint =
        this->FindGripperJoint(this->cableConnection0LinkEntity, _ecm);
    if (gripperJoint != kNullEntity) {
      _ecm.RequestRemoveEntity(gripperJoint);
      gzdbg << "Manually detached end 0" << std::endl;
    }
  }

  if (this->attachEnd1Requested.exchange(false)) {
    if (this->FindGripperJoint(this->cableConnection1LinkEntity, _ecm) ==
        kNullEntity) {
      Entity jointEntity = _ecm.CreateEntity();
      _ecm.CreateComponent(jointEntity,
                           components::DetachableJoint(
                               {this->endEffectorLinkEntity,
                                this->cableConnection1LinkEntity, "fixed"}));
      gzdbg << "Manually attached end 1" << std::endl;
    }
  }
  if (this->detachEnd1Requested.exchange(false)) {
    Entity gripperJoint =
        this->FindGripperJoint(this->cableConnection1LinkEntity, _ecm);
    if (gripperJoint != kNullEntity) {
      _ecm.RequestRemoveEntity(gripperJoint);
      gzdbg << "Manually detached end 1" << std::endl;
    }
  }
}

//////////////////////////////////////////////////
void CableModeratorPlugin::MakeCableStatic(
    size_t _cableIndex, gz::sim::EntityComponentManager& _ecm) {
  // Skip if this is the cable that was just completed, as its connectors
  // are already frozen.
  if (this->nextCableIndex > 0 && (this->nextCableIndex - 1) == _cableIndex) {
    return;
  }

  Model(this->cableTrackers[_cableIndex].modelEntity)
      .SetStaticStateCmd(_ecm, true);
}

//////////////////////////////////////////////////
void CableModeratorPlugin::MakeCableDynamic(
    size_t _cableIndex, gz::sim::EntityComponentManager& _ecm) {
  Model(this->cableTrackers[_cableIndex].modelEntity)
      .SetStaticStateCmd(_ecm, false);
}

//////////////////////////////////////////////////
void CableModeratorPlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                                     gz::sim::EntityComponentManager& _ecm) {
  if (this->cableState == CableState::COMPLETED) return;

  if (!this->foundAllCables) {
    this->foundAllCables = this->FindCableModels(_info, _ecm);
  }

  for (size_t i = 1; i < this->cableTrackers.size(); ++i) {
    auto& tracker = this->cableTrackers[i];
    // Skip the active cable (which has index this->nextCableIndex - 1)
    if (tracker.found && !tracker.frozen && i + 1 != this->nextCableIndex) {
      auto timeSinceFound = std::chrono::duration_cast<std::chrono::seconds>(
          _info.simTime - tracker.foundTime);
      if (timeSinceFound.count() >= 0.0) {
        this->MakeCableStatic(i, _ecm);
        tracker.frozen = true;
        gzmsg << "Froze cable " << this->cableConfigs[i].modelName << std::endl;
      }
    }
  }

  if (this->endEffectorLinkEntity == kNullEntity) {
    this->endEffectorLinkEntity =
        findLinkInModel(this->endEffectorModelName, endEffectorLinkName, _ecm);
  }
  if (this->endEffectorLinkEntity == kNullEntity) return;

  this->ProcessManualGraspRequests(_ecm);

  if (this->cableState == CableState::HARNESS) {
    // Hold both connections of the cable in place
    this->detachableJointStatic0Entity =
        this->MakeStatic(this->cableConnection0LinkEntity, true, _ecm);
    this->detachableJointStatic1Entity =
        this->MakeStatic(this->cableConnection1LinkEntity, true, _ecm);

    gzmsg << "Cable transitioning to WAITING_CONN_0 state." << std::endl;
    this->cableState = CableState::WAITING_CONN_0;
  }

  if (this->cableState == CableState::WAITING_CONN_0) {
    this->activeGraspJoint = this->FindExternalGraspJoint(_ecm);
    if (this->activeGraspJoint != kNullEntity) {
      // External skill has grasped the cable - remove the static hold
      if (this->detachableJointStatic0Entity != kNullEntity) {
        _ecm.RequestRemoveEntity(this->detachableJointStatic0Entity);
        this->detachableJointStatic0Entity = kNullEntity;
      }
      gzmsg << "Cable transitioning to ATTACHED_TO_GRIPPER_CONN_0 state."
            << std::endl;
      this->cableState = CableState::ATTACHED_TO_GRIPPER_CONN_0;
    }
  }

  if (this->cableState == CableState::ATTACHED_TO_GRIPPER_CONN_0) {
    if (this->cableConnectionPortSubs.empty()) {
      this->CreatePortSubscribers(
          this->cableConfigs[this->nextCableIndex - 1].connection0PortName);
    }

    if (this->attachCableConnectionToPort) {
      this->cableConnectionPortSubs.clear();
      this->attachCableConnectionToPort = false;
      this->touchEventCallbackNamespace = std::nullopt;

      gzmsg << "Cable transitioning to ATTACH_TO_PORT_CONN_0 state."
            << std::endl;
      this->cableState = CableState::ATTACH_TO_PORT_CONN_0;
    }
  }

  if (this->cableState == CableState::ATTACH_TO_PORT_CONN_0) {
    if (this->activeGraspJoint == kNullEntity ||
        !_ecm.HasEntity(this->activeGraspJoint)) {
      this->detachableJointStatic0Entity =
          this->MakeStatic(this->cableConnection0LinkEntity, true, _ecm);

      this->activeGraspJoint = kNullEntity;
      gzmsg << "Cable transitioning to WAITING_CONN_1 state." << std::endl;
      this->cableState = CableState::WAITING_CONN_1;
    }
  }

  if (this->cableState == CableState::WAITING_CONN_1) {
    this->activeGraspJoint = this->FindExternalGraspJoint(_ecm);
    if (this->activeGraspJoint != kNullEntity) {
      // External skill has grasped the cable - remove the static hold
      if (this->detachableJointStatic1Entity != kNullEntity) {
        _ecm.RequestRemoveEntity(this->detachableJointStatic1Entity);
        this->detachableJointStatic1Entity = kNullEntity;
      }
      gzmsg << "Cable transitioning to ATTACHED_TO_GRIPPER_CONN_1 state."
            << std::endl;
      this->cableState = CableState::ATTACHED_TO_GRIPPER_CONN_1;
    }
  }

  if (this->cableState == CableState::ATTACHED_TO_GRIPPER_CONN_1) {
    if (this->cableConnectionPortSubs.empty()) {
      this->CreatePortSubscribers(
          this->cableConfigs[this->nextCableIndex - 1].connection1PortName);
    }

    if (this->attachCableConnectionToPort) {
      this->cableConnectionPortSubs.clear();
      this->attachCableConnectionToPort = false;
      this->touchEventCallbackNamespace = std::nullopt;

      gzmsg << "Cable transitioning to ATTACH_TO_PORT_CONN_1 state."
            << std::endl;
      this->cableState = CableState::ATTACH_TO_PORT_CONN_1;
    }
  }

  if (this->cableState == CableState::ATTACH_TO_PORT_CONN_1) {
    if (this->activeGraspJoint == kNullEntity ||
        !_ecm.HasEntity(this->activeGraspJoint)) {
      this->detachableJointStatic1Entity =
          this->MakeStatic(this->cableConnection1LinkEntity, true, _ecm);

      this->activeGraspJoint = kNullEntity;
      gzmsg << "Cable transitioning to NEXT_CABLE state." << std::endl;
      this->cableState = CableState::NEXT_CABLE;
    }
  }

  if (this->cableState == CableState::NEXT_CABLE) {
    this->cableConnectionPortSubs.clear();
    this->attachCableConnectionToPort = false;
    this->touchEventCallbackNamespace = std::nullopt;

    // Freeze all links in the completed cable before proceeding
    if (this->nextCableIndex > 0) {
      this->MakeCableStatic(this->nextCableIndex - 1, _ecm);
    }

    if (this->nextCableIndex < this->cableTrackers.size()) {
      if (this->ToggleActiveCable(_ecm)) {
        gzmsg << "Cable transitioning to HARNESS state for next cable."
              << std::endl;
        this->cableState = CableState::HARNESS;
      } else {
        gzmsg << "Failed to toggle active cable. Transitioning to COMPLETED "
                 "state."
              << std::endl;
        this->cableState = CableState::COMPLETED;
      }
    } else {
      gz::msgs::StringMsg msg;
      msg.set_data("all_cables_completed");
      this->taskCompletionPub.Publish(msg);

      gzmsg << "All cables processed. Transitioning to COMPLETED state."
            << std::endl;
      this->cableState = CableState::COMPLETED;
    }
  }
}

//////////////////////////////////////////////////
void CableModeratorPlugin::Update(const gz::sim::UpdateInfo& /*_info*/,
                                  gz::sim::EntityComponentManager& /*_ecm*/) {}

//////////////////////////////////////////////////
void CableModeratorPlugin::PostUpdate(
    const gz::sim::UpdateInfo& /*_info*/,
    const gz::sim::EntityComponentManager& /*_ecm*/) {}

//////////////////////////////////////////////////
void CableModeratorPlugin::Reset(const gz::sim::UpdateInfo& /*_info*/,
                                 gz::sim::EntityComponentManager& /*_ecm*/) {
  gzdbg << "aic_gazebo::CableModeratorPlugin::Reset" << std::endl;
}

//////////////////////////////////////////////////
void CableModeratorPlugin::Cleanup(gz::sim::EntityComponentManager& _ecm) {
  if (this->detachableJointStatic0Entity != kNullEntity)
    _ecm.RequestRemoveEntity(this->detachableJointStatic0Entity);
  if (this->detachableJointStatic1Entity != kNullEntity)
    _ecm.RequestRemoveEntity(this->detachableJointStatic1Entity);

  for (const auto& ent : this->staticEntities) {
    if (ent != kNullEntity) {
      _ecm.RequestRemoveEntity(ent);
    }
  }

  this->staticEntities.clear();
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

  return detachableJointEntity;
}

//////////////////////////////////////////////////
Entity CableModeratorPlugin::FindGripperJoint(
    gz::sim::Entity _connectionLinkEntity,
    const gz::sim::EntityComponentManager& _ecm) const {
  Entity result = kNullEntity;
  _ecm.Each<components::DetachableJoint>(
      [&](const Entity& _entity,
          const components::DetachableJoint* _joint) -> bool {
        const auto& info = _joint->Data();
        // Check if this joint connects the end-effector to the connection link
        // (in either parent/child order)
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
    const gz::sim::EntityComponentManager& _ecm) const {
  if (this->cableModel.Entity() == kNullEntity) return kNullEntity;

  auto cableLinks = this->cableModel.Links(_ecm);
  std::unordered_set<Entity> cableLinkSet(cableLinks.begin(), cableLinks.end());

  Entity result = kNullEntity;
  _ecm.Each<components::DetachableJoint>(
      [&](const Entity& _entity,
          const components::DetachableJoint* _joint) -> bool {
        const auto& info = _joint->Data();

        bool parentInCable = cableLinkSet.count(info.parentLink) > 0;
        bool childInCable = cableLinkSet.count(info.childLink) > 0;

        // We are looking for a joint between a cable link and a non-cable link
        if (parentInCable != childInCable) {
          Entity externalLink =
              parentInCable ? info.childLink : info.parentLink;

          // Check if this is a plugin-created static hold
          // These connect to a model containing "__static__"
          auto parentEnt =
              _ecm.Component<components::ParentEntity>(externalLink);
          if (parentEnt) {
            auto modelName =
                _ecm.Component<components::Name>(parentEnt->Data());
            if (modelName &&
                modelName->Data().find("__static__") != std::string::npos) {
              return true;  // Skip plugin-created static holds
            }
          }

          // If we are here, it's an external joint
          result = _entity;
          return false;  // Found it
        }
        return true;
      });
  return result;
}

//////////////////////////////////////////////////
bool CableModeratorPlugin::FindCableModels(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager& _ecm) {
  bool allFound = true;
  for (size_t i = 0; i < this->cableConfigs.size(); ++i) {
    if (!this->cableTrackers[i].found) {
      auto entitiesMatchingName =
          entitiesFromScopedName(this->cableConfigs[i].modelName, _ecm);
      if (entitiesMatchingName.size() == 1) {
        this->cableTrackers[i].modelEntity = *entitiesMatchingName.begin();
        this->cableTrackers[i].found = true;
        this->cableTrackers[i].foundTime = _info.simTime;
        gzdbg << "Found cable model " << this->cableConfigs[i].modelName
              << std::endl;
      } else {
        // gzwarn << "Cable model " << this->cableConfigs[i].modelName << "
        // could not be found.\n";
        allFound = false;
      }
    }
  }

  return allFound;
}

//////////////////////////////////////////////////
void CableModeratorPlugin::CreatePortSubscribers(const std::string& _portName) {
  std::vector<std::string> allTopics;
  this->node.TopicList(allTopics);

  std::function<void(const msgs::Boolean&, const transport::MessageInfo&)>
      callback = [this](const msgs::Boolean& _msg,
                        const transport::MessageInfo& _info) {
        size_t pos = _info.Topic().rfind("/");
        // TODO(luca) If we only ever receive true boolean messages consider
        // removing the atomic bool and only having the namespace as a
        // std::optional
        // TODO(luca) Protect the namespace with a mutex since it can't be
        // atomic and it is set in a separate thread.
        // TODO(luca) Check if we need the namespace at all, it is used to check
        // if we plugged into the right port but this might be done by another
        // plugin (scoring)
        this->touchEventCallbackNamespace = _info.Topic().substr(0, pos);
        this->attachCableConnectionToPort = _msg.data();
        gzdbg << "Cable connection touched: " << _msg.data()
              << ". Topic: " << _info.Topic() << std::endl;
      };

  for (const auto& topic : allTopics) {
    if (topic.find(_portName) != std::string::npos &&
        topic.find("touched") != std::string::npos) {
      this->cableConnectionPortSubs.emplace_back(
          this->node.CreateSubscriber(topic, callback));
    }
  }
}

//////////////////////////////////////////////////
bool CableModeratorPlugin::ToggleActiveCable(
    gz::sim::EntityComponentManager& _ecm) {
  // Make sure we unfreeze the newly active cable
  this->MakeCableDynamic(this->nextCableIndex, _ecm);
  // this->cableTrackers[this->nextCableIndex].frozen = false;

  this->cableModel =
      Model(this->cableTrackers[this->nextCableIndex].modelEntity);
  this->activeGraspJoint = kNullEntity;
  const auto cableModelName = this->cableModel.Name(_ecm);

  if (!this->cableModel.Valid(_ecm)) return false;

  const auto& config = this->cableConfigs[this->nextCableIndex];

  this->cableConnection0LinkEntity =
      findLinkInModel(cableModelName, config.connection0LinkName, _ecm);

  this->cableConnection1LinkEntity =
      findLinkInModel(cableModelName, config.connection1LinkName, _ecm);

  if (this->cableConnection0LinkEntity == kNullEntity ||
      this->cableConnection1LinkEntity == kNullEntity)
    return false;

  this->manualGraspSubs.clear();

  auto cbAttach0 = std::function<void(const gz::msgs::Empty&)>(
      [this](const gz::msgs::Empty&) { this->attachEnd0Requested = true; });
  auto cbDetach0 = std::function<void(const gz::msgs::Empty&)>(
      [this](const gz::msgs::Empty&) { this->detachEnd0Requested = true; });
  auto cbAttach1 = std::function<void(const gz::msgs::Empty&)>(
      [this](const gz::msgs::Empty&) { this->attachEnd1Requested = true; });
  auto cbDetach1 = std::function<void(const gz::msgs::Empty&)>(
      [this](const gz::msgs::Empty&) { this->detachEnd1Requested = true; });

  this->manualGraspSubs.emplace_back(this->node.CreateSubscriber(
      "/" + cableModelName + "/attach_end_0", cbAttach0));
  this->manualGraspSubs.emplace_back(this->node.CreateSubscriber(
      "/" + cableModelName + "/detach_end_0", cbDetach0));
  this->manualGraspSubs.emplace_back(this->node.CreateSubscriber(
      "/" + cableModelName + "/attach_end_1", cbAttach1));
  this->manualGraspSubs.emplace_back(this->node.CreateSubscriber(
      "/" + cableModelName + "/detach_end_1", cbDetach1));

  this->nextCableIndex++;

  return true;
}

}  // namespace aic_gazebo
