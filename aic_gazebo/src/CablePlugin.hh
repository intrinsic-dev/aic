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

    /// \brief Waiting for end-effector / port to be ready
    WAITING,

    /// \brief Create connections with end-effector / port
    CREATE_CONNECTIONS,

    /// \brief Cable connection 0 is attached to gripper
    CABLE_ATTACHED_TO_GRIPPER,

    /// \brief Attach cable connection 0 to port
    ATTACH_CABLE_TO_PORT,

    /// \brief Task complete - cable connection 0 is connected to port
    COMPLETED,

    /// \brief Cable model is removed.
    CABLE_REMOVED,
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

    /// \brief Check if model entity is removed
    /// \param[in] _ecm Immutable reference to the Entity Component Manager.
    private: bool IsModelValid(const gz::sim::EntityComponentManager& _ecm);

    /// \brief Clean up entities created by this plugin
    /// \param[in] _ecm Mutable reference to the Entity Component Manager.
    private: void Cleanup(gz::sim::EntityComponentManager& _ecm);

    /// \brief Make an entity static by spawning a static model and attaching
    /// the entity to a static model
    /// \param[in] _attachEntityAsParentOfJoint True to attach entity as parent of
    /// the detachable joint.
    /// \param[in] _creator Sdf entity creator for creating a static model
    /// \param[in] _ecm Entity component manager
    private: gz::sim::Entity MakeStatic(gz::sim::Entity _entity,
                             bool _attachEntityAsParentOfJoint,
                             gz::sim::SdfEntityCreator* _creator,
                             gz::sim::EntityComponentManager& _ecm);

    /// \brief Spawn an invisible collision to block the gap of the gripper fingers
    /// in order to preent the cable body from swinging between that gap.
    /// \param[in] _pose Pose to spawn the cable guard in world frame
    /// \param[in] _creator Sdf entity creator for creating a static model
    /// \param[in] _ecm Entity component manager
    private: gz::sim::Entity SpawnCableGuard(const gz::math::Pose3d& _pose,
                                             gz::sim::SdfEntityCreator* _creator,
                                             gz::sim::EntityComponentManager& _ecm);

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

    /// \brief Detachable joint entity for cable guard entity.
    private: gz::sim::Entity detachableJointCableGuardEntity{
        gz::sim::kNullEntity};

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

    /// \brief Delay in seconds for creating the connection joints.
    private: double createJointDelay{0.0};

    /// \brief Delay in seconds for locking end-effector after insertion.
    private: double lockEndEffectorDelay{0.0};

    /// \brief Start time for delay in creating connection joints.
    private: double createJointDelayStartTime{0.0};

    /// \brief Start time for delay in locking end effector after insertion.
    private: double lockEndEffectorDelayStartTime{0.0};

    /// \brief Sdf entity creator for spawning static entities
    /// Used for holding cable connections in place
    private: std::unique_ptr<gz::sim::SdfEntityCreator> creator{nullptr};

    /// \brief Current state of the cable
    private: CableState cableState{CableState::INITIALIZATION};

    /// \brief Name of the cable connection 0 port topic
    private: std::unordered_set<std::string> cableConnection0PortTopics;

    /// \brief Cable connection 0 port subscribers
    private: std::vector<gz::transport::Node::Subscriber>
        cableConnection0PortSubs;

    /// \brief Task completion event publisher
    private: gz::transport::Node::Publisher taskCompletionPub;

    /// \brief Whether to attach cable connection 0 to port
    /// This is set on cableConnection0PortSub callback
    private: std::atomic<bool> attachCableConnection0ToPort{false};

    /// \brief Topic in which the touch event is received
    private: std::string touchEventCallbackNamespace;

    /// \brief Gazebo transport node
    private: gz::transport::Node node;

    /// \brief Static entities created by this plugin
    private: std::unordered_set<gz::sim::Entity> staticEntities;

    /// \brief Pose offset of cable guard w.r.t. end-effector.
    private: gz::math::Pose3d cableGuardOffsetFromEndEffector;

    /// \brief Whether to spawn cable guard
    /// This is an invisible collision that stops the cable body for moving
    /// through the gap between the gripper fingers
    private: bool spawnCableGuard = false;
};
}
#endif
