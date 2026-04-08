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

#ifndef AIC_GAZEBO__CABLE_MODERATOR_PLUGIN_HH_
#define AIC_GAZEBO__CABLE_MODERATOR_PLUGIN_HH_

#include <atomic>
#include <chrono>
#include <unordered_set>

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

    /// \brief Waiting for end effector to approach connection 0
    WAITING_CONN_0,

    /// \brief Connection 0 attached to gripper, wait for touch
    ATTACHED_TO_GRIPPER_CONN_0,

    /// \brief Detach connection 0 from gripper, make static
    ATTACH_TO_PORT_CONN_0,

    /// \brief Waiting for end effector to approach connection 1
    WAITING_CONN_1,

    /// \brief Connection 1 attached to gripper, wait for touch
    ATTACHED_TO_GRIPPER_CONN_1,

    /// \brief Detach connection 1 from gripper, make static
    ATTACH_TO_PORT_CONN_1,

    /// \brief Move to the next cable
    NEXT_CABLE,

    /// \brief Task complete - cable connections are completed
    COMPLETED,
  };

  /// \brief Configuration for a cable
  struct CableConfig {
    /// \brief Name of the cable model
    std::string modelName;

    /// \brief Name of the cable connection 0 link
    std::string connection0LinkName;

    /// \brief Name of the cable connection 0 port
    std::string connection0PortName;

    /// \brief Name of the cable connection 1 link
    std::string connection1LinkName;

    /// \brief Name of the cable connection 1 port
    std::string connection1PortName;
  };

  /// \brief Plugin for initializing the cable
  /// It waits for end-effector / port to be ready before creating connections
  /// with them using detachable joints.
  class CableModeratorPlugin:
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

    /// \brief Check if model entity is removed
    /// \param[in] _ecm Immutable reference to the Entity Component Manager.
    private: bool IsModelValid(const gz::sim::EntityComponentManager& _ecm);

    /// \brief Clean up entities created by this plugin
    /// \param[in] _ecm Mutable reference to the Entity Component Manager.
    private: void Cleanup(gz::sim::EntityComponentManager& _ecm);

    /// \brief Make an entity static by spawning a static model and attaching
    /// the entity to a static model
    /// \param[in] _attachEntityAsParentOfJoint True to attach entity as parent
    /// of the detachable joint.
    /// \param[in] _ecm Entity component manager
    private: gz::sim::Entity MakeStatic(gz::sim::Entity _entity,
                             bool _attachEntityAsParentOfJoint,
                             gz::sim::EntityComponentManager& _ecm);

    /// \brief Check end effector distance to connection and handle grasping
    /// \param[in] _connectionLinkEntity The connection link to check
    /// \param[in, out] _detachableJointStaticEntity The static joint holding
    ///  it (removed if grasped)
    /// \param[in] _ecm Entity Component Manager
    /// \return True if grasped, false otherwise
    private: bool HandleGrasping(gz::sim::Entity _connectionLinkEntity,
                                 gz::sim::Entity& _detachableJointStaticEntity,
                                 gz::sim::EntityComponentManager& _ecm);

    /// \brief Toggle active cable. Done by setting internal vairables to keep
    /// track of the connection link entities of the next cable in the queue
    /// \param[in] _ecm Entity Component Manager
    private: bool ToggleActiveCable(
        const gz::sim::EntityComponentManager& _ecm);

    /// \brief Find Cable model entities based on their names
    /// \param[in] _ecm Entity Component Manager
    private: bool FindCableModels(const gz::sim::EntityComponentManager& _ecm);

    /// \brief Initialize port contact subscribers for the given port
    /// \param[in] _portName The name of the port to subscribe to for contacts
    private: void CreatePortSubscribers(const std::string& _portName);

    /// \brief Entity of attachment link in the end effector model
    private: gz::sim::Entity endEffectorLinkEntity{gz::sim::kNullEntity};

    /// \brief Connection 0 link entity in the cable model
    private: gz::sim::Entity cableConnection0LinkEntity{gz::sim::kNullEntity};

    /// \brief Connection 1 link entity in the cable model
    private: gz::sim::Entity cableConnection1LinkEntity{gz::sim::kNullEntity};

    /// \brief Detachable joint entity for gripper connection
    private: gz::sim::Entity detachableJointGripperConnEntity{
        gz::sim::kNullEntity};

    /// \brief Detachable joint entity for making cable connection 0 static
    private: gz::sim::Entity detachableJointStatic0Entity{gz::sim::kNullEntity};

    /// \brief Detachable joint entity for making cable connection 1 static
    private: gz::sim::Entity detachableJointStatic1Entity{gz::sim::kNullEntity};

    /// \brief A list of cable configurations
    private: std::vector<CableConfig> cableConfigs;

    /// \brief The current active cable model.
    private: gz::sim::Model cableModel;

    /// \brief Index of cable model that is currently active.
    private: std::size_t cableIndex{0u};

    /// \brief Entity offset for end effector
    private: gz::math::Pose3d endEffectorOffset;

    /// \brief Entities of the cable models
    private: std::vector<gz::sim::Entity> cableModels;

    /// \brief Name of the end effector model
    private: std::string endEffectorModelName;

    /// \brief Name of the end effector link
    private: std::string endEffectorLinkName;

    /// \brief Distance threshold for grasping the cable connection
    private: double graspDistanceThreshold{0.05};

    /// \brief Sdf entity creator for spawning static entities
    /// Used for holding cable connections in place
    private: std::unique_ptr<gz::sim::SdfEntityCreator> creator{nullptr};

    /// \brief Current state of the cable
    private: CableState cableState{CableState::INITIALIZATION};

    /// \brief Cable connection port subscribers
    private: std::vector<gz::transport::Node::Subscriber>
        cableConnectionPortSubs;

    /// \brief Task completion event publisher
    private: gz::transport::Node::Publisher taskCompletionPub;

    /// \brief Whether to attach cable connection to the port
    /// This is set on cableConnectionPortSub callback
    private: std::atomic<bool> attachCableConnectionToPort{false};

    /// \brief Topic in which the touch event is received
    /// This is set on cableConnectionPortSub callback
    private: std::optional<std::string> touchEventCallbackNamespace;

    /// \brief Gazebo transport node
    private: gz::transport::Node node;

    /// \brief Static entities created by this plugin
    private: std::unordered_set<gz::sim::Entity> staticEntities;
};
}
#endif
