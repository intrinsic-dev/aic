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
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <queue>
#include <unordered_map>

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

  this->cableInsertionPub = this->node.Advertise<gz::msgs::StringMsg_V>(
      "/cable_moderator/insertion_event");

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
  Model(tracker.modelEntity).SetStatic(_ecm, true);

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
  Model(tracker.modelEntity).SetStatic(_ecm, false);
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
  this->FindCableModels(_ecm);

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
    Entity graspJoint = this->FindExternalGraspJoint(tracker, _ecm);
    if (graspJoint != kNullEntity) {
      // Create subscribers if they weren't already.
      if (tracker.portSubs.empty()) this->CreatePortSubscribers(i);

      auto closerEnd = this->FindGraspedEnd(i, graspJoint, _ecm);
      if (!closerEnd.has_value()) {
        gzwarn << "Grasped end not found. Report this" << std::endl;
        continue;
      }
      if (tracker.activeGraspJoint.load() == kNullEntity ||
          closerEnd != tracker.lastGraspedEnd) {
        gzmsg << "Cable " << this->cableConfigs[i].modelName
              << " grasp confirmed near End " << *closerEnd
              << " (joint: " << graspJoint << ")" << std::endl;
        tracker.lastGraspedEnd = closerEnd;
        if (tracker.activeGraspJoint.load() == kNullEntity)
          this->MakeCableDynamic(i, _ecm);
      }

      if (closerEnd == 0) {
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
      tracker.activeGraspJoint.store(graspJoint);
    } else {
      if (tracker.activeGraspJoint.exchange(kNullEntity) != kNullEntity) {
        this->MakeCableStatic(i, _ecm);
      }
      tracker.lastGraspedEnd = std::nullopt;
    }

    if (tracker.end0Inserted &&
        tracker.detachableJointStatic0Entity == kNullEntity) {
      tracker.detachableJointStatic0Entity =
          this->MakeStatic(tracker.connection0LinkEntity, true, _ecm);
    }
    if (tracker.end1Inserted &&
        tracker.detachableJointStatic1Entity == kNullEntity) {
      tracker.detachableJointStatic1Entity =
          this->MakeStatic(tracker.connection1LinkEntity, true, _ecm);
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
    const CableTracker& _tracker, const EntityComponentManager& _ecm) const {
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
                modelName->Data().find("__static__") != std::string::npos)
              return true;
          }
          result = _entity;
          return false;
        }
        return true;
      });
  return result;
}

//////////////////////////////////////////////////
void CableModeratorPlugin::FindCableModels(const EntityComponentManager& _ecm) {
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

        this->MakeCableStatic(i, const_cast<EntityComponentManager&>(_ecm));
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
      gz::msgs::StringMsg_V pubMsg;
      pubMsg.add_data(this->cableConfigs[_cableIndex].modelName);
      pubMsg.add_data(std::to_string(end));
      pubMsg.add_data(tracker.touchEventCallbackNamespace);
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

}  // namespace aic_gazebo
