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

#ifndef AIC_GAZEBO__OFFLIMITCONTACTS_PLUGIN_HH_
#define AIC_GAZEBO__OFFLIMITCONTACTS_PLUGIN_HH_

#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace aic_gazebo
{
  /// \brief A plugin to detect off-limit contacts.
  class OffLimitContactsPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventManager) override;

    // Documentation inherited
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Parse the SDF parameters.
    /// \param[in] _sdf Pointer to the SDF element.
    /// \return True if success or false otherwise.
    private: bool ParseSDF(sdf::ElementPtr _sdf);

    /// \brief Create ContactSensorData components for all the collision
    /// elements within modelEntity.
    /// \param[in out] _ecm The Entity Component Manager.
    private: void CreateCollisionData(gz::sim::EntityComponentManager &_ecm);

    /// \brief Populate the offLimitEntities collection with the entities
    /// associated to offLimitModelNames.
    /// \param[in out] _ecm The Entity Component Manager.
    /// \return True if all the entities were found and populated,
    /// false otherwise.
    private: bool InitializeOffLimitEntities(
      gz::sim::EntityComponentManager &_ecm);

    /// \brief The topic to publish contacts information.
    private: std::string topic;

    /// \brief A transport node.
    private: gz::transport::Node node;

    /// \brief A transport publisher.
    private: gz::transport::Node::Publisher publisher;

    /// \brief The model entity which this plugin checks collisions for
    private: gz::sim::Entity modelEntity = gz::sim::kNullEntity;

    /// \brief The name of the model that this plugin checks collisions for
    private: std::string modelName;

    /// \brief Collection of model names considered off-limits.
    private: std::set<std::string> offLimitModelNames;

    /// \brief Collection of entities considered off-limits.
    private: std::set<gz::sim::Entity> offLimitEntities;

    /// \brief System update period calculated from <update_rate>.
    private: std::chrono::steady_clock::duration updatePeriod{0};

    /// \brief Last system update simulation time.
    private: std::chrono::steady_clock::duration lastUpdateTime{0};
  };
}
#endif
