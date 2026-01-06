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

#include <gz/msgs/boolean.pb.h>

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
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/World.hh>

using namespace gz;
using namespace sim;

GZ_ADD_PLUGIN(aic_gazebo::CablePlugin, gz::sim::System,
              aic_gazebo::CablePlugin::ISystemConfigure,
              aic_gazebo::CablePlugin::ISystemPreUpdate,
              aic_gazebo::CablePlugin::ISystemUpdate,
              aic_gazebo::CablePlugin::ISystemPostUpdate,
              aic_gazebo::CablePlugin::ISystemReset)

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
    gzwarn << "Model " << _modelName << " could not be found.\n";
  }
  return kNullEntity;
}

/// \brief Make an entity static by spawning a static model and attaching
/// the entity to a static model
/// \param[in] _attachEntityAsParentOfJoint True to attach entity as parent of
/// the detachable joint.
/// \param[in] _creator_ Sdf entity creator for creating a static model
/// \param[in] _ecm Entity component manager
Entity makeStatic(Entity _entity, bool _attachEntityAsParentOfJoint,
                  SdfEntityCreator* _creator, EntityComponentManager& _ecm) {
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

  auto nameComp = _ecm.Component<components::Name>(_entity);
  std::string staticEntName = nameComp->Data() + "__static__";
  Entity staticEntity =
      _ecm.EntityByComponents(components::Name(staticEntName));
  if (staticEntity == kNullEntity) {
    staticModelToSpawn.SetName(staticEntName);
    staticEntity = _creator->CreateEntities(&staticModelToSpawn);
    _creator->SetParent(staticEntity,
                        _ecm.EntityByComponents(components::World()));
  }

  Entity staticLinkEntity = _ecm.EntityByComponents(
      components::Link(), components::ParentEntity(staticEntity),
      components::Name("static_link"));

  if (staticLinkEntity == kNullEntity) return detachableJointEntity;

  Entity parentLinkEntity;
  Entity childLinkEntity;
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

}  // namespace

namespace aic_gazebo {

//////////////////////////////////////////////////
void CablePlugin::Configure(const gz::sim::Entity& _entity,
                            const std::shared_ptr<const sdf::Element>& _sdf,
                            gz::sim::EntityComponentManager& _ecm,
                            gz::sim::EventManager& _eventManager) {
  gzdbg << "aic_gazebo::CablePlugin::Configure on entity: " << _entity
        << std::endl;

  this->model = Model(_entity);
  if (!this->model.Valid(_ecm)) {
    gzerr << "CablePlugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }

  this->cableModelName = this->model.Name(_ecm);

  if (_sdf->HasElement("cable_connection_0_link")) {
    this->cableConnection0LinkName =
        _sdf->Get<std::string>("cable_connection_0_link");
  } else {
    gzerr << "Missing <cable_connection_0_link> parameter." << std::endl;
    return;
  }

  if (_sdf->HasElement("cable_connection_0_port")) {
    this->cableConnection0PortName =
        _sdf->Get<std::string>("cable_connection_0_port");
  } else {
    gzerr << "Missing <cable_connection_0_port> parameter." << std::endl;
    return;
  }

  if (_sdf->HasElement("cable_connection_1_link")) {
    this->cableConnection1LinkName =
        _sdf->Get<std::string>("cable_connection_1_link");
  } else {
    gzerr << "Missing <cable_connection_1_link> parameter." << std::endl;
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

  double delay = _sdf->Get<double>("create_connection_delay_s", 0.0).first;
  this->createJointDelay = std::chrono::duration<double>(delay);
  this->creator = std::make_unique<SdfEntityCreator>(_ecm, _eventManager);

  gzmsg << "Cable transitioning to HARNESS state." << std::endl;
  this->cableState = CableState::HARNESS;
}

//////////////////////////////////////////////////
void CablePlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                            gz::sim::EntityComponentManager& _ecm) {
  if (this->cableConnection0LinkEntity == kNullEntity) {
    this->cableConnection0LinkEntity =
        findLinkInModel(this->cableModelName, cableConnection0LinkName, _ecm);
  }

  if (this->cableConnection1LinkEntity == kNullEntity) {
    this->cableConnection1LinkEntity =
        findLinkInModel(this->cableModelName, cableConnection1LinkName, _ecm);
  }

  if (this->endEffectorLinkEntity == kNullEntity) {
    this->endEffectorLinkEntity =
        findLinkInModel(this->endEffectorModelName, endEffectorLinkName, _ecm);
  }

  if (this->endEffectorLinkEntity == kNullEntity ||
      this->cableConnection0LinkEntity == kNullEntity ||
      this->cableConnection1LinkEntity == kNullEntity)
    return;

  if (this->cableState == CableState::HARNESS) {
    // Hold both connections of the cable in place
    this->detachableJointStatic0Entity = makeStatic(
        this->cableConnection0LinkEntity, false, this->creator.get(), _ecm);
    this->detachableJointStatic1Entity = makeStatic(
        this->cableConnection1LinkEntity, true, this->creator.get(), _ecm);

    gzmsg << "Cable transitioning to WAITING state." << std::endl;
    this->cableState = CableState::WAITING;
  }

  if (cableState == CableState::WAITING) {
    // Wait for specified delay duration before making connecting
    // cable to gripper
    if (_info.simTime < this->createJointDelay) {
      return;
    }

    gzmsg << "Cable transitioning to CREATE_CONNECTIONS state." << std::endl;
    this->cableState = CableState::CREATE_CONNECTIONS;
  }

  if (cableState == CableState::CREATE_CONNECTIONS) {
    // Detach joint that's holding cable connection 0 in place
    if (this->detachableJointStatic0Entity != kNullEntity) {
      _ecm.RequestRemoveEntity(this->detachableJointStatic0Entity);
      this->detachableJointStatic0Entity = kNullEntity;
      return;
    }

    // Attach cable connection 0 to end effector
    if (this->detachableJoint0Entity == kNullEntity) {
      this->detachableJoint0Entity = _ecm.CreateEntity();
      _ecm.CreateComponent(this->detachableJoint0Entity,
                           components::DetachableJoint(
                               {this->endEffectorLinkEntity,
                                this->cableConnection0LinkEntity, "fixed"}));
    }

    gzmsg << "Cable transitioning to CABLE_ATTACHED_TO_GRIPPER state."
          << std::endl;
    this->cableState = CableState::CABLE_ATTACHED_TO_GRIPPER;
  }

  if (this->cableState == CableState::CABLE_ATTACHED_TO_GRIPPER) {
    if (this->cableConnection0PortTopic.empty()) {
      std::vector<std::string> allTopics;
      this->node.TopicList(allTopics);

      for (const auto& topic : allTopics) {
        if (topic.find(this->cableConnection0PortName) != std::string::npos) {
          this->cableConnection0PortTopic = topic;
          break;
        }
      }

      if (this->cableConnection0PortTopic.empty()) return;

      std::function<void(const msgs::Boolean&)> callback =
          [this](const msgs::Boolean& _msg) {
            this->attachCableConnection0ToPort = _msg.data();
            gzdbg << "Cable connection 0 touched: " << _msg.data() << std::endl;
          };
      this->cableConnection0PortSub = this->node.CreateSubscriber(
          this->cableConnection0PortTopic, callback);
    }

    if (this->attachCableConnection0ToPort) {
      gzmsg << "Cable transitioning to ATTACH_CABLE_TO_PORT state."
            << std::endl;
      this->cableState = CableState::ATTACH_CABLE_TO_PORT;
    }
  }

  if (this->cableState == CableState::ATTACH_CABLE_TO_PORT) {
    // Detach all connection joints first
    if (this->detachableJoint0Entity != kNullEntity) {
      _ecm.RequestRemoveEntity(this->detachableJoint0Entity);
      this->detachableJoint0Entity = kNullEntity;
      _ecm.RequestRemoveEntity(this->detachableJointStatic1Entity);
      this->detachableJointStatic1Entity = kNullEntity;
      return;
    }

    // Attach cable connection 0 to port
    // Simulate this by making all cable connections static
    this->detachableJointStatic0Entity = makeStatic(
        this->cableConnection0LinkEntity, true, this->creator.get(), _ecm);
    this->detachableJointStatic1Entity = makeStatic(
        this->cableConnection1LinkEntity, true, this->creator.get(), _ecm);
    gzmsg << "Cable transitioning to COMPLETED state." << std::endl;
    this->cableState = CableState::COMPLETED;
  }
}

//////////////////////////////////////////////////
void CablePlugin::Update(const gz::sim::UpdateInfo& /*_info*/,
                         gz::sim::EntityComponentManager& /*_ecm*/) {}

//////////////////////////////////////////////////
void CablePlugin::PostUpdate(const gz::sim::UpdateInfo& /*_info*/,
                             const gz::sim::EntityComponentManager& /*_ecm*/) {}

//////////////////////////////////////////////////
void CablePlugin::Reset(const gz::sim::UpdateInfo& /*_info*/,
                        gz::sim::EntityComponentManager& /*_ecm*/) {
  gzdbg << "aic_gazebo::CablePlugin::Reset" << std::endl;
}
}  // namespace aic_gazebo
