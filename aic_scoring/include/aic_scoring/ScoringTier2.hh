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

#include <chrono>
#include <string>

#include <gz/math/Pose3.hh>

#ifndef AIC_SCORING__SCORING_TIER2_HH_
#define AIC_SCORING__SCORING_TIER2_HH_

namespace aic_scoring
{
  /// \brief Tier2 POD.
  class Pluggable
  {
    /// \brief Plug/port name.
    public: std::string name;

    /// \brief Plug/port type.
    public: std::string type;

    /// \brief Position.
    public: gz::math::Vector3d position;
  };

  // The Tier2 scoring interface.
  class ScoringTier2 : public rclcpp::Node
  {
    /// \brief Class constructor.
    public: ScoringTier2();

    /// \brief Populate the scoring input params from a YAML file.
    /// \param[in] _yamlFile Input YAML file.
    public: bool ParseStats(const std::string &_yamlFile);

    /// \brief Store the current distance cable-connector.
    public: void Update();

    /// \brief All pluggable plugs.
    public: std::map<std::string, Pluggable> plugs;

    /// \brief All pluggable ports.
    public: std::map<std::string, Pluggable> ports;

    /// \brief Plug<->port connections.
    /// The first key is always the plug, followed by "&", follower by port.
    public: std::map<std::string, double> pluggableMap;
  };
}
#endif
