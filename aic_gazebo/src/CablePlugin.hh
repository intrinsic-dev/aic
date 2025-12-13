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

#ifndef AIC_GAZEBO__CABLE_PLUGIN_HH_
#define AIC_GAZEBO__CABLE_PLUGIN_HH_

#include <chrono>

#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace aic_gazebo
{
  // The main AIC scoring plugin.
  class CablePlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemUpdate,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemReset
  {
    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_element,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventManager) override;

    // Documentation inherited
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void Update(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                          const gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void Reset(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Entity of attachment link in the parent end effector model
    private: gz::sim::Entity endEffectorLinkEntity{gz::sim::kNullEntity};

    /// \brief Entity of attachment link in the child cable model
    private: gz::sim::Entity cableLinkEntity{gz::sim::kNullEntity};

    /// \brief Entity of the detachable joint created by this system
    private: gz::sim::Entity detachableJointEntity{gz::sim::kNullEntity};

    /// \brief The model associated with this system.
    private: gz::sim::Model model;

    /// \brief Name of the cable model
    private: std::string cableModelName;

    /// \brief Name of the cable link
    private: std::string cableConnectionLinkName;

    /// \brief Name of the end effector link
    private: std::string endEffectorModelName;

    /// \brief Name of the end effector link
    private: std::string endEffectorConnectionLinkName;

    /// \brief Connection local pose offset relative to parent.
    private: gz::math::Pose3d connectionLocalOffset;

    private: bool moveCableToEndEffector{false};
    private: bool createConnectionJoint{false};
    private: std::chrono::steady_clock::duration createJointDelay{0};
  };
}
#endif
