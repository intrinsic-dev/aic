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

#ifndef AIC_GAZEBO__WORLDSDFGENERATOR_PLUGIN_HH_
#define AIC_GAZEBO__WORLDSDFGENERATOR_PLUGIN_HH_

#include <chrono>
#include <string>

#include <gz/transport/Node.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>

namespace aic_gazebo
{
  /// \brief Plugin for generating and saving world to SDF
  class WorldSdfGeneratorPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
  {
    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_element,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventManager) override;

    // Documentation inherited
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                          const gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Gazebo transport node
    private: gz::transport::Node node;

    /// \brief Whether sdf has been generated or not
    private: bool sdfGenerated = false;

    /// \brief Local path of where to save the file
    private: std::string saveWorldPath;

    /// \brief Local path of where to save the file
    private: std::chrono::duration<double> saveWorldDelay;
};
}
#endif
