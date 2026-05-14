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
#include <vector>
#include <deque>
#include <string>
#include <memory>

#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>

namespace aic_gazebo
{

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

  /// \brief Tracker for each cable model state
  struct CableTracker {
    /// \brief Entity of the cable model
    gz::sim::Entity modelEntity{gz::sim::kNullEntity};
    /// \brief Whether the cable has been found in simulation
    bool found{false};
    /// \brief Detachable joint entity for making cable connection 0 static
    gz::sim::Entity detachableJointStatic0Entity{gz::sim::kNullEntity};
    /// \brief Detachable joint entity for making cable connection 1 static
    gz::sim::Entity detachableJointStatic1Entity{gz::sim::kNullEntity};
    /// \brief Connection 0 link entity in the cable model
    gz::sim::Entity connection0LinkEntity{gz::sim::kNullEntity};
    /// \brief Connection 1 link entity in the cable model
    gz::sim::Entity connection1LinkEntity{gz::sim::kNullEntity};

    /// \brief Whether connection 0 has been inserted into a port
    bool end0Inserted{false};
    /// \brief Whether connection 1 has been inserted into a port
    bool end1Inserted{false};
    /// \brief Whether connection 0 insertion has been published
    bool end0Published{false};
    /// \brief Whether connection 1 insertion has been published
    bool end1Published{false};
    /// \brief Whether the cable task is completed
    bool isCompleted{false};
    /// \brief Entity of the detachable joint found while waiting for grasp
    std::atomic<gz::sim::Entity> activeGraspJoint{gz::sim::kNullEntity};
    /// \brief Topic namespace from the touched port
    std::string touchEventCallbackNamespace;
    /// \brief Which end was last identified as grasped (0, 1)
    std::optional<int> lastGraspedEnd;
    /// \brief Cable connection port subscribers
    std::vector<gz::transport::Node::Subscriber> portSubs;

    /// \brief Topological graph of the cable (link -> neighbor links)
    std::unordered_map<gz::sim::Entity, std::vector<gz::sim::Entity>> neighbors;

    /// \brief Flags for manual attach/detach of connection 0
    std::atomic<bool> attachEnd0Requested{false};
    std::atomic<bool> detachEnd0Requested{false};

    /// \brief Flags for manual attach/detach of connection 1
    std::atomic<bool> attachEnd1Requested{false};
    std::atomic<bool> detachEnd1Requested{false};
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

    /// \brief Clean up entities created by this plugin
    /// \param[in] _ecm Mutable reference to the Entity Component Manager.
    private: void Cleanup(gz::sim::EntityComponentManager& _ecm);

    /// \brief Make an entity static by spawning a static model and attaching
    /// the entity to a static model
    /// \param[in] _entity The entity to make static
    /// \param[in] _attachEntityAsParentOfJoint True to attach entity as parent
    /// of the detachable joint.
    /// \param[in] _ecm Entity component manager
    private: gz::sim::Entity MakeStatic(gz::sim::Entity _entity,
                             bool _attachEntityAsParentOfJoint,
                             gz::sim::EntityComponentManager& _ecm);

    /// \brief Find a detachable joint created by an external plugin that
    /// connects the end-effector link to the given connection link.
    /// \param[in] _connectionLinkEntity The cable connection link to check
    /// \param[in] _ecm Entity Component Manager
    /// \return Entity of the gripper joint if found, kNullEntity otherwise
    private: gz::sim::Entity FindGripperJoint(
        gz::sim::Entity _connectionLinkEntity,
        const gz::sim::EntityComponentManager& _ecm) const;

    /// \brief Find a detachable joint created by an external plugin that
    /// connects any link in the given cable model to an external link.
    /// \param[in] _tracker The cable tracker to check
    /// \param[in] _ecm Entity Component Manager
    /// \return Entity of the grasp joint if found, kNullEntity otherwise
    private: gz::sim::Entity FindExternalGraspJoint(
        const CableTracker& _tracker,
        const gz::sim::EntityComponentManager& _ecm) const;

    /// \brief Process any pending manual attach/detach requests
    /// \param[in] _ecm Entity Component Manager
    private: void ProcessManualGraspRequests(
        gz::sim::EntityComponentManager& _ecm);

    /// \brief Freezes the cable model by making it static
    /// \param[in] _cableIndex The index of the cable to freeze
    /// \param[in] _ecm Entity Component Manager
    private: void MakeCableStatic(size_t _cableIndex,
        gz::sim::EntityComponentManager& _ecm);

    /// \brief Unfreezes the cable model by making it dynamic
    /// \param[in] _cableIndex The index of the cable to unfreeze
    /// \param[in] _ecm Entity Component Manager
    private: void MakeCableDynamic(size_t _cableIndex,
        gz::sim::EntityComponentManager& _ecm);

    /// \brief Find which end of the cable the gripper is closer to
    /// \param[in] _cableIndex The index of the cable
    /// \param[in] _graspJoint The entity of the detachable joint representing the grasp
    /// \param[in] _ecm Entity Component Manager
    /// \return 0 for end 0, 1 for end 1, std::nullopt if cannot be determined
    private: std::optional<int> FindGraspedEnd(size_t _cableIndex,
        gz::sim::Entity _graspJoint,
        const gz::sim::EntityComponentManager& _ecm) const;

    /// \brief Find Cable model entities based on their names
    /// \param[in] _ecm Entity Component Manager
    private: void FindCableModels(const gz::sim::EntityComponentManager& _ecm);

    /// \brief Initialize port contact subscribers for the given cable
    /// \param[in] _cableIndex The index of the cable to subscribe for
    private: void CreatePortSubscribers(size_t _cableIndex);

    /// \brief Entity of attachment link in the end effector model
    private: gz::sim::Entity endEffectorLinkEntity{gz::sim::kNullEntity};

    /// \brief A list of cable configurations
    private: std::vector<CableConfig> cableConfigs;

    /// \brief State trackers for the cable models
    private: std::deque<CableTracker> cableTrackers;

    /// \brief Index of the cable currently being manually tested
    private: int cableIndex{0};

    /// \brief Name of the end effector model
    private: std::string endEffectorModelName;

    /// \brief Name of the end effector link
    private: std::string endEffectorLinkName;

    /// \brief Sdf entity creator for spawning static entities
    /// Used for holding cable connections in place
    private: std::unique_ptr<gz::sim::SdfEntityCreator> creator{nullptr};

    /// \brief Cable insertion event publisher
    private: gz::transport::Node::Publisher cableInsertionPub;

    /// \brief Gazebo transport node
    private: gz::transport::Node node;

    /// \brief Static entities created by this plugin
    private: std::unordered_set<gz::sim::Entity> staticEntities;

    /// \brief Manual grasp subscribers
    private: std::vector<gz::transport::Node::Subscriber> manualGraspSubs;

  };
}
#endif
