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

/// \brief Local pose offset of the cable guard w.r.t. the end effector.
/// This is tuned for the robotiq hand-e.
const gz::math::Pose3d kCableGuardOffsetForRobotiqHandE =
    math::Pose3d(0, 0.0118, 0.165, 0, 0, 0);

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

  this->cableGuardOffsetFromEndEffector =
      _sdf->Get<math::Pose3d>("cable_guard_offset_from_end_effector",
                              kCableGuardOffsetForRobotiqHandE)
          .first;

  this->spawnCableGuard = _sdf->Get<bool>("spawn_cable_guard", false).first;

  double delay = _sdf->Get<double>("create_connection_delay_s", 0.0).first;
  this->createJointDelay = delay;
  this->creator = std::make_unique<SdfEntityCreator>(_ecm, _eventManager);

  this->taskCompletionPub = this->node.Advertise<gz::msgs::StringMsg>(
      "/" + this->cableModelName + "/insertion_event");

  gzmsg << "Cable transitioning to HARNESS state." << std::endl;
  this->cableState = CableState::HARNESS;
}

//////////////////////////////////////////////////
void CablePlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                            gz::sim::EntityComponentManager& _ecm) {
  if (this->cableState == CableState::CABLE_REMOVED) return;

  if (!this->IsModelValid(_ecm)) {
    this->Cleanup(_ecm);

    gzmsg << "Cable transitioning to CABLE_REMOVED state." << std::endl;
    this->cableState = CableState::CABLE_REMOVED;
    return;
  }

  if (this->cableState == CableState::COMPLETED) {
    return;
  }

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
    this->detachableJointStatic0Entity = this->MakeStatic(
        this->cableConnection0LinkEntity, true, this->creator.get(), _ecm);
    this->detachableJointStatic1Entity = this->MakeStatic(
        this->cableConnection1LinkEntity, true, this->creator.get(), _ecm);

    this->createJointDelayStartTime =
        std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count();

    gzmsg << "Cable transitioning to WAITING state." << std::endl;
    this->cableState = CableState::WAITING;
  }

  if (this->cableState == CableState::WAITING) {
    // Wait for specified delay duration before making connecting
    // cable to gripper
    double timeNow =
        std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count();
    if (timeNow - this->createJointDelayStartTime < this->createJointDelay)
      return;

    gzmsg << "Cable transitioning to CREATE_CONNECTIONS state." << std::endl;
    this->cableState = CableState::CREATE_CONNECTIONS;
  }

  if (this->cableState == CableState::CREATE_CONNECTIONS) {
    // Detach joints that are holding cable connections in place
    if (this->detachableJointStatic0Entity != kNullEntity ||
        this->detachableJointStatic1Entity != kNullEntity) {
      _ecm.RequestRemoveEntity(this->detachableJointStatic0Entity);
      _ecm.RequestRemoveEntity(this->detachableJointStatic1Entity);
      this->detachableJointStatic0Entity = kNullEntity;
      this->detachableJointStatic1Entity = kNullEntity;
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

    if (this->spawnCableGuard) {
      auto endEffectorWorldPose =
          gz::sim::worldPose(this->endEffectorLinkEntity, _ecm);
      auto cableGuardPose =
          endEffectorWorldPose * this->cableGuardOffsetFromEndEffector;
      this->detachableJointCableGuardEntity =
          this->SpawnCableGuard(cableGuardPose, this->creator.get(), _ecm);
      gzmsg << "Spawning Cable Guard." << std::endl;
    }

    gzmsg << "Cable transitioning to CABLE_ATTACHED_TO_GRIPPER state."
          << std::endl;
    this->cableState = CableState::CABLE_ATTACHED_TO_GRIPPER;
  }

  if (this->cableState == CableState::CABLE_ATTACHED_TO_GRIPPER) {
    if (this->cableConnection0PortTopics.empty()) {
      std::vector<std::string> allTopics;
      this->node.TopicList(allTopics);

      for (const auto& topic : allTopics) {
        if (topic.find(this->cableConnection0PortName) != std::string::npos) {
          this->cableConnection0PortTopics.insert(topic);
        }
      }

      if (this->cableConnection0PortTopics.empty()) return;

      std::function<void(const msgs::Boolean&, const transport::MessageInfo&)>
          callback = [this](const msgs::Boolean& _msg,
                            const transport::MessageInfo& _info) {
            size_t pos = _info.Topic().rfind("/");
            this->touchEventCallbackNamespace = _info.Topic().substr(0, pos);
            this->attachCableConnection0ToPort = _msg.data();
            gzdbg << "Cable connection 0 touched: " << _msg.data()
                  << ". Topic: " << _info.Topic() << std::endl;
          };
      for (const auto& topic : this->cableConnection0PortTopics) {
        this->cableConnection0PortSubs.emplace_back(
            this->node.CreateSubscriber(topic, callback));
      }
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
      return;
    }

    // Attach cable connection 0 to port
    // Simulate this by making all cable connections static
    if (this->detachableJointStatic0Entity == kNullEntity ||
        this->detachableJointStatic1Entity == kNullEntity) {
      this->detachableJointStatic0Entity = this->MakeStatic(
          this->cableConnection0LinkEntity, true, this->creator.get(), _ecm);
      this->detachableJointStatic1Entity = this->MakeStatic(
          this->cableConnection1LinkEntity, true, this->creator.get(), _ecm);
      this->lockEndEffectorDelayStartTime =
          std::chrono::duration_cast<std::chrono::seconds>(_info.simTime)
              .count();
    }

    double timeNow =
        std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count();
    if (timeNow - this->lockEndEffectorDelayStartTime <
        this->lockEndEffectorDelay)
      return;

    // Lock end-effector in place to simulate gripping the cable that was
    // inserted into the port.
    // Note: creating detachable joints between the gripper and cable after the
    // the cable is made static (after insertion) causes jerky motion on the
    // robot arm as if the controller is fighting against the joints.
    // So workaround this by locking the end-effector in place (make it static).
    this->detachableJoint0Entity = this->MakeStatic(
        this->endEffectorLinkEntity, true, this->creator.get(), _ecm);

    gz::msgs::StringMsg msg;
    msg.set_data(this->touchEventCallbackNamespace);
    this->taskCompletionPub.Publish(msg);
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

//////////////////////////////////////////////////
bool CablePlugin::IsModelValid(const gz::sim::EntityComponentManager& _ecm) {
  bool modelValid = true;
  _ecm.EachRemoved<components::Model>(
      [&](const Entity& _entity, const components::Model*) -> bool {
        if (_entity == this->model.Entity()) {
          modelValid = false;
          return false;
        }
        return true;
      });

  return modelValid;
}

//////////////////////////////////////////////////
void CablePlugin::Cleanup(gz::sim::EntityComponentManager& _ecm) {
  // Clean up detachable joints
  if (this->detachableJointStatic0Entity != kNullEntity)
    _ecm.RequestRemoveEntity(this->detachableJointStatic0Entity);
  if (this->detachableJointStatic1Entity != kNullEntity)
    _ecm.RequestRemoveEntity(this->detachableJointStatic1Entity);
  if (this->detachableJoint0Entity != kNullEntity)
    _ecm.RequestRemoveEntity(this->detachableJoint0Entity);
  if (this->detachableJoint1Entity != kNullEntity)
    _ecm.RequestRemoveEntity(this->detachableJoint1Entity);
  if (this->detachableJointCableGuardEntity != kNullEntity)
    _ecm.RequestRemoveEntity(this->detachableJointCableGuardEntity);

  for (const auto& ent : this->staticEntities) _ecm.RequestRemoveEntity(ent);

  this->staticEntities.clear();
}

//////////////////////////////////////////////////
Entity CablePlugin::MakeStatic(Entity _entity,
                               bool _attachEntityAsParentOfJoint,
                               SdfEntityCreator* _creator,
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

  auto nameComp = _ecm.Component<components::Name>(_entity);
  std::string staticEntName = nameComp->Data() + "__static__";
  Entity staticEntity =
      _ecm.EntityByComponents(components::Name(staticEntName));
  if (staticEntity == kNullEntity) {
    staticModelToSpawn.SetName(staticEntName);
    staticEntity = _creator->CreateEntities(&staticModelToSpawn);
    this->staticEntities.insert(staticEntity);
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

//////////////////////////////////////////////////
Entity CablePlugin::SpawnCableGuard(const math::Pose3d& _pose,
                                    SdfEntityCreator* _creator,
                                    EntityComponentManager& _ecm) {
  std::stringstream modelStr;
  modelStr << R"(<?xml version="1.0"?>
  <sdf version="1.11">
    <model name="cable_guard">
      <pose>)"
           << _pose << R"(</pose>
      <link name="box_link">
        <inertial>
          <inertia>
            <ixx>1e-6</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1e-6</iyy>
            <iyz>0</iyz>
            <izz>1e-6</izz>
          </inertia>
          <mass>0.001</mass>
        </inertial>
        <collision name="box_collision">
          <geometry>
            <box>
              <size>0.024 0.001 0.005</size>
            </box>
          </geometry>
        </collision>
        <!-- Uncomment below to see the visual for debugging -->
        <!--
        <visual name="box_visual">
          <geometry>
            <box>
              <size>0.024 0.001 0.005</size>
            </box>
          </geometry>
          <material>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>1 1 1 1</specular>
          </material>
       </visual>
       -->
      </link>
    </model>
  </sdf>
  )";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(modelStr.str());
  Entity modelEntity = _creator->CreateEntities(root.Model());
  _creator->SetParent(modelEntity,
                      _ecm.EntityByComponents(components::World()));
  this->staticEntities.insert(modelEntity);
  Entity modelLinkEntity = _ecm.EntityByComponents(
      components::ParentEntity(modelEntity), components::Name("box_link"));
  Entity detachableJointEntity = _ecm.CreateEntity();
  _ecm.CreateComponent(detachableJointEntity,
                       components::DetachableJoint({this->endEffectorLinkEntity,
                                                    modelLinkEntity, "fixed"}));
  return detachableJointEntity;
}

}  // namespace aic_gazebo
