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

#include "OffLimitContactsPlugin.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>

using namespace gz;
using namespace sim;

GZ_ADD_PLUGIN(aic_gazebo::OffLimitContactsPlugin, gz::sim::System,
              aic_gazebo::OffLimitContactsPlugin::ISystemConfigure,
              aic_gazebo::OffLimitContactsPlugin::ISystemPreUpdate)

namespace aic_gazebo {
//////////////////////////////////////////////////
void OffLimitContactsPlugin::Configure(
    const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &, EventManager & /*_eventManager*/) {
  gzdbg << "aic_gazebo::OffLimitContactsPlugin::Configure on entity: "
        << _entity << std::endl;

  if (!this->ParseSDF(_sdf->Clone())) return;

  this->publisher = this->node.Advertise<gz::msgs::Contacts>(this->topic);
  if (!this->publisher) {
    gzerr << "Error advertising topic [" << this->topic << "]" << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
void OffLimitContactsPlugin::PreUpdate(const UpdateInfo &_info,
                                       EntityComponentManager &_ecm) {
  // Throttle update rate.
  auto elapsed = _info.simTime - this->lastUpdateTime;
  if (elapsed > std::chrono::steady_clock::duration::zero() &&
      elapsed < this->updatePeriod) {
    return;
  }
  this->lastUpdateTime = _info.simTime;

  // Find the robot model entity
  if (this->modelEntity == kNullEntity) {
    this->modelEntity = _ecm.EntityByComponents(
        components::Model(), components::Name(this->modelName));

    if (this->modelEntity == kNullEntity) return;

    // Enable contacts for collisions in this model
    this->CreateCollisionData(_ecm);
  }

  // Try to resolve any remaining off-limit model names to entities.
  // Don't return early, detect contacts with already-resolved entities
  // even while waiting for others (e.g. task_board spawned mid-trial).
  if (this->offLimitModelNames.size() != this->offLimitEntities.size()) {
    this->InitializeOffLimitEntities(_ecm);
  }

  if (this->offLimitEntities.empty()) {
    return;
  }

  bool shouldPublish = false;
  _ecm.Each<components::ContactSensorData>(
      [&](const Entity &,
          const components::ContactSensorData *_contacts) -> bool {
        // Only consider contacts with off-limit models (e.g. enclosure).
        for (const auto &contact : _contacts->Data().contact()) {
          shouldPublish = ((this->offLimitEntities.find(topLevelModel(
                                contact.collision1().id(), _ecm)) !=
                            this->offLimitEntities.end()) &&
                           (topLevelModel(contact.collision2().id(), _ecm) ==
                            this->modelEntity)) ||
                          ((this->offLimitEntities.find(topLevelModel(
                                contact.collision2().id(), _ecm)) !=
                            this->offLimitEntities.end()) &&
                           (topLevelModel(contact.collision1().id(), _ecm) ==
                            this->modelEntity));

          if (shouldPublish) break;
        }
        if (shouldPublish) this->publisher.Publish(_contacts->Data());

        return true;
      });
}

//////////////////////////////////////////////////
bool OffLimitContactsPlugin::ParseSDF(sdf::ElementPtr _sdf) {
  // Initialize system update period.
  double rate = _sdf->Get<double>("update_rate", 1).first;
  std::chrono::duration<double> period{rate > 0 ? 1 / rate : 0};
  this->updatePeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

  this->topic =
      _sdf->Get<std::string>("topic", "/aic/gazebo/contacts/off_limit").first;

  if (!_sdf->HasElement("model")) {
    gzerr << "Unable to find <model> element in SDF." << std::endl;
    return false;
  }
  this->modelName = _sdf->Get<std::string>("model");
  if (this->modelName.empty()) {
    gzerr << "Model name can not be an empty string." << std::endl;
    return false;
  }

  if (!_sdf->HasElement("off_limit_models")) {
    gzerr << "Unable to find <off_limit_models> element in SDF." << std::endl;
    return false;
  }
  auto offLimitModelsElem = _sdf->GetElement("off_limit_models");

  // We need at least one model.
  if (!offLimitModelsElem->HasElement("model")) {
    gzerr << "Unable to find <model> element in <off_limit_models>."
          << std::endl;
    return false;
  }
  auto modelElem = offLimitModelsElem->GetElement("model");
  while (modelElem) {
    this->offLimitModelNames.insert(modelElem->Get<std::string>());
    modelElem = modelElem->GetNextElement("model");
  }

  return true;
}

//////////////////////////////////////////////////
void OffLimitContactsPlugin::CreateCollisionData(EntityComponentManager &_ecm) {
  // Get the name of the world
  std::string worldName;
  _ecm.Each<components::World, components::Name>(
      [&](const Entity &, const components::World *,
          const components::Name *_name) -> bool {
        // We assume there's only one world
        worldName = _name->Data();
        return false;
      });

  // Enable contacts for all the model collisions.
  _ecm.Each<components::Collision>(
      [&](const Entity &_entity, const components::Collision *) -> bool {
        auto parentEntity = topLevelModel(_entity, _ecm);
        if (parentEntity != this->modelEntity) return true;

        // Check if ContactSensorData has already been created
        bool collisionHasContactSensor = _ecm.EntityHasComponentType(
            _entity, components::ContactSensorData::typeId);

        if (collisionHasContactSensor) {
          gzdbg << "ContactSensorData detected in collision [" << _entity << "]"
                << std::endl;
          return true;
        }

        _ecm.CreateComponent(_entity, components::ContactSensorData());
        gzdbg << "Enabled collision [" << _entity << "]" << std::endl;
        return true;
      });
}

//////////////////////////////////////////////////
bool OffLimitContactsPlugin::InitializeOffLimitEntities(
    EntityComponentManager &_ecm) {
  for (const auto &modelName : this->offLimitModelNames) {
    Entity entity = kNullEntity;
    auto entitiesMatchingName = entitiesFromScopedName(modelName, _ecm);
    // Filter for entities with only models
    std::vector<Entity> candidateEntities;
    std::copy_if(entitiesMatchingName.begin(), entitiesMatchingName.end(),
                 std::back_inserter(candidateEntities), [&_ecm](Entity e) {
                   return _ecm.EntityHasComponentType(
                       e, components::Model::typeId);
                 });

    if (candidateEntities.size() == 1) {
      entity = *candidateEntities.begin();
    } else if (candidateEntities.size() > 1) {
      gzwarn << "Off-limit model name '" << modelName
             << "' matched multiple entities. Skipping." << std::endl;
    }
    if (entity == kNullEntity) {
      return false;
    }

    this->offLimitEntities.insert(entity);
  }
  return true;
}
}  // namespace aic_gazebo
