
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

#include "aic_gazebo/ScoringPlugin.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensor.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>

#include "proto/scoring.pb.h"

GZ_ADD_PLUGIN(aic_gazebo::ScoringPlugin, gz::sim::System,
              aic_gazebo::ScoringPlugin::ISystemConfigure,
              aic_gazebo::ScoringPlugin::ISystemPreUpdate,
              aic_gazebo::ScoringPlugin::ISystemReset)

namespace aic_gazebo {
//////////////////////////////////////////////////
void ScoringPlugin::Configure(const gz::sim::Entity& _entity,
                              const std::shared_ptr<const sdf::Element>& _sdf,
                              gz::sim::EntityComponentManager& /*_ecm*/,
                              gz::sim::EventManager& /*_eventManager*/) {
  gzdbg << "aic_gazebo::ScoringPlugin::Configure on entity: " << _entity
        << std::endl;

  // Initialize system update period.
  double rate = _sdf->Get<double>("update_rate", 1).first;
  std::chrono::duration<double> period{rate > 0 ? 1 / rate : 0};
  this->updatePeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

  this->topic = _sdf->Get<std::string>("topic", "/aic/gazebo/data").first;
  this->pub = this->node.Advertise<msgs::Scoring>(this->topic);
  if (!pub) {
    std::cerr << "Error advertising topic [" << this->topic << "]" << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
void ScoringPlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                              gz::sim::EntityComponentManager& /*_ecm*/) {
  // Throttle update rate.
  auto elapsed = _info.simTime - this->lastUpdateTime;
  if (elapsed > std::chrono::steady_clock::duration::zero() &&
      elapsed < this->updatePeriod) {
    return;
  }
  this->lastUpdateTime = _info.simTime;

  this->scoringMsg.mutable_header()->mutable_stamp()->CopyFrom(
      gz::sim::convert<gz::msgs::Time>(_info.simTime));

  if (!this->pub.Publish(this->scoringMsg)) return;
}

//////////////////////////////////////////////////
void ScoringPlugin::Reset(const gz::sim::UpdateInfo& /*_info*/,
                          gz::sim::EntityComponentManager& /*_ecm*/) {
  gzdbg << "aic_gazebo::ScoringPlugin::Reset" << std::endl;
}
}  // namespace aic_gazebo
