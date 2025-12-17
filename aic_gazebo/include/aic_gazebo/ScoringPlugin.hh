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

#ifndef AIC_GAZEBO__SCORING_PLUGIN_HH_
#define AIC_GAZEBO__SCORING_PLUGIN_HH_

#include "proto/scoring.pb.h"
#include <chrono>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace aic_gazebo
{
  // The main AIC scoring plugin.
  class ScoringPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemReset
  {
    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventManager) override;

    // Documentation inherited
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void Reset(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Scoring message.
    private: msgs::Scoring scoringMsg;

    /// \brief A transport node.
    private: gz::transport::Node node;

    /// \brief A transport publisher.
    private: gz::transport::Node::Publisher pub;

    /// \brief The topic to publish scoring information.
    private: std::string topic;

    /// \brief System update period calculated from <update_rate>.
    private: std::chrono::steady_clock::duration updatePeriod{0};

    /// \brief Last system update simulation time.
    private: std::chrono::steady_clock::duration lastUpdateTime{0};
  };
}
#endif
