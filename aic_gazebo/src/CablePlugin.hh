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

#include <atomic>
#include <chrono>

#include <gz/transport/Node.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>

namespace aic_gazebo
{
  /// \brief State of the cable
  enum class CableState {
    /// \brief Harnessed to the world, i.e. made static
    /// before creating connections
    INITIALIZATION,

    /// \brief Harnessed to the world, i.e. made static
    /// before creating connections
    HARNESS,

    /// \brief Waiting for end-effector / port to be ready
    WAITING,

    /// \brief Create connections with end-effector / port
    CREATE_CONNECTIONS,

    /// \brief Cable connection 0 is attached to gripper
    CABLE_ATTACHED_TO_GRIPPER,

    /// \brief Attach cable connection 0 to port
    ATTACH_CABLE_TO_PORT,

    /// \brief Task complete - cable 0 is connected to port
    COMPLETED,
  };

  /// \brief Plugin for initializing the cable
  /// It waits for end-effector / port to be ready before creating connections
  /// with them using detachable joints.
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

    /// \brief Entity of attachment link in the end effector model
    private: gz::sim::Entity endEffectorLinkEntity{gz::sim::kNullEntity};

    /// \brief Connection 0 link entity in the cable model
    private: gz::sim::Entity cableConnection0LinkEntity{gz::sim::kNullEntity};

    /// \brief Connection 1 link entity in the cable model
    private: gz::sim::Entity cableConnection1LinkEntity{gz::sim::kNullEntity};

    /// \brief Detachable joint entity for connection 0
    private: gz::sim::Entity detachableJoint0Entity{gz::sim::kNullEntity};

    /// \brief Detachable joint entity for connection 1
    private: gz::sim::Entity detachableJoint1Entity{gz::sim::kNullEntity};

    /// \brief Detachable joint entity for making cable connection 0 static
    private: gz::sim::Entity detachableJointStatic0Entity{gz::sim::kNullEntity};

    /// \brief Detachable joint entity for making cable connection 1 static
    private: gz::sim::Entity detachableJointStatic1Entity{gz::sim::kNullEntity};

    /// \brief The model associated with this system.
    private: gz::sim::Model model;

    /// \brief Name of the cable model
    private: std::string cableModelName;

    /// \brief Name of the cable connection 0 link
    private: std::string cableConnection0LinkName;

    /// \brief Name of the cable connection 0 port
    private: std::string cableConnection0PortName;

    /// \brief Name of the cable connection 1 link
    private: std::string cableConnection1LinkName;

    /// \brief Name of the end effector model
    private: std::string endEffectorModelName;

    /// \brief Name of the end effector link
    private: std::string endEffectorLinkName;

    /// \brief Name of the target model for connection 1
    private: std::string connection1ModelName;

    /// \brief Delay for creating the connection joints.
    private: std::chrono::duration<double> createJointDelay{0};

    /// \brief Sdf entity creator for spawning static entities
    /// Used for holding cable connections in place
    private: std::unique_ptr<gz::sim::SdfEntityCreator> creator{nullptr};

    /// \brief Current state of the cable
    private: CableState cableState{CableState::INITIALIZATION};

    /// \brief Name of the cable connection 0 port topic
    private: std::string cableConnection0PortTopic;

    /// \brief Cable connection 0 port subscriber
    private: gz::transport::Node::Subscriber cableConnection0PortSub;

    /// \brief Whether to attach cable connection 0 to port
    /// This is set on cableConnection0PortSub callback
    private: std::atomic<bool> attachCableConnection0ToPort{false};

    /// \brief Gazebo transport node
    private: gz::transport::Node node;
};
}
#endif
