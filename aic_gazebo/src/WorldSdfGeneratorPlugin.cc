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

#include "WorldSdfGeneratorPlugin.hh"

#include <gz/msgs/sdf_generator_config.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <fstream>
#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>

using namespace gz;
using namespace sim;

GZ_ADD_PLUGIN(aic_gazebo::WorldSdfGeneratorPlugin, gz::sim::System,
              aic_gazebo::WorldSdfGeneratorPlugin::ISystemConfigure,
              aic_gazebo::WorldSdfGeneratorPlugin::ISystemPostUpdate)

namespace {
inline constexpr char kLocalSaveWorldPath[] = "/tmp/aic.sdf";
}

namespace aic_gazebo {

//////////////////////////////////////////////////
void WorldSdfGeneratorPlugin::Configure(
    const gz::sim::Entity&, const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager&, gz::sim::EventManager&) {
  gzdbg << "aic_gazebo::WorldSdfGeneratorPlugin::Configure" << std::endl;

  double delay = _sdf->Get<double>("save_world_delay_s", 0.0).first;
  this->saveWorldDelay = std::chrono::duration<double>(delay);
  this->saveWorldPath =
      _sdf->Get<std::string>("save_world_path", kLocalSaveWorldPath).first;
}

//////////////////////////////////////////////////
void WorldSdfGeneratorPlugin::PostUpdate(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager& _ecm) {
  if (this->sdfGenerated) return;

  // Wait for specified delay duration before requesting to save world
  if (_info.simTime < this->saveWorldDelay) return;

  Entity world = _ecm.EntityByComponents(components::World());
  auto nameComp = _ecm.Component<components::Name>(world);
  std::string worldName = nameComp->Data();
  const std::string sdfGenService{std::string("/world/") + worldName +
                                  "/generate_world_sdf"};
  msgs::StringMsg genWorldSdf;
  msgs::SdfGeneratorConfig req;
  auto* globalConfig = req.mutable_global_entity_gen_config();
  msgs::Set(globalConfig->mutable_expand_include_tags(), true);

  const unsigned int timeout{5000};
  bool result = false;
  bool serviceCall =
      this->node.Request(sdfGenService, req, timeout, genWorldSdf, result);
  if (serviceCall && result && !genWorldSdf.data().empty()) {
    gzdbg << "Saving world: " << worldName << " to: " << this->saveWorldPath
          << std::endl;
    std::ofstream fs(this->saveWorldPath, std::ios::out);
    if (fs.is_open()) {
      fs << genWorldSdf.data();
      gzmsg << "World saved to " << this->saveWorldPath << std::endl;
    } else {
      gzmsg << "File: " << this->saveWorldPath << " could not be opened for "
            << "saving. Please check that the directory containg the "
            << "file exists and the correct permissions are set." << std::endl;
    }
  } else {
    if (!serviceCall) {
      gzmsg << "Service call for generating world SDF timed out" << std::endl;
    }
    gzmsg << "Unknown error occured when saving the world." << std::endl;
  }

  this->sdfGenerated = true;
}
}  // namespace aic_gazebo
