
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

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include "aic_gazebo/ScoringPlugin.hh"

GZ_ADD_PLUGIN(
    aic_gazebo::ScoringPlugin,
    gz::sim::System,
    aic_gazebo::ScoringPlugin::ISystemConfigure,
    aic_gazebo::ScoringPlugin::ISystemPreUpdate,
    aic_gazebo::ScoringPlugin::ISystemUpdate,
    aic_gazebo::ScoringPlugin::ISystemPostUpdate,
    aic_gazebo::ScoringPlugin::ISystemReset
)

namespace aic_gazebo
{
//////////////////////////////////////////////////
void ScoringPlugin::Configure(const gz::sim::Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &/*_element*/,
                        gz::sim::EntityComponentManager &/*_ecm*/,
                        gz::sim::EventManager &/*_eventManager*/)
{
  gzdbg << "aic_gazebo::ScoringPlugin::Configure on entity: " << _entity
        << std::endl;
}

//////////////////////////////////////////////////
void ScoringPlugin::PreUpdate(const gz::sim::UpdateInfo &/*_info*/,
                              gz::sim::EntityComponentManager &/*_ecm*/)
{
}

//////////////////////////////////////////////////
void ScoringPlugin::Update(const gz::sim::UpdateInfo &/*_info*/,
                           gz::sim::EntityComponentManager &/*_ecm*/)
{
}

//////////////////////////////////////////////////
void ScoringPlugin::PostUpdate(const gz::sim::UpdateInfo &/*_info*/,
                               const gz::sim::EntityComponentManager &/*_ecm*/)
{
}

//////////////////////////////////////////////////
void ScoringPlugin::Reset(const gz::sim::UpdateInfo &/*_info*/,
                          gz::sim::EntityComponentManager &/*_ecm*/)
{
  gzdbg << "aic_gazebo::ScoringPlugin::Reset" << std::endl;
}
}  // namespace aic_gazebo
