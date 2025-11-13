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

#ifndef AIC_SCORING__SCORING_TIER2_HH_
#define AIC_SCORING__SCORING_TIER2_HH_

namespace aic_scoring
{
  /// \brief Tier2 POD.
  class StatsTier2
  {
    /// \brief Timestamp.
    public: std::chrono::time_point<std::chrono::steady_clock> timestamp;

    /// \brief Distance cable-connector in meters.
    public: double distance;

    /// \brief Whether the connector and cable are plugged in.
    public: bool connected = false;
  };
  
  // The Tier2 scoring interface.
  class ScoringTier2
  {
    /// \brief Class constructor.
    public: ScoringTier2();

    /// \brief Populate the scoring input params from a YAML file.
    /// \param[in] _yamlFile Input YAML file.
    public: bool ParseStats(const std::string &_yamlFile);

    /// \brief Check distance between the tip of the cable and the connector.
    /// \return Distance (m) between the cable and connector.
    public: virtual double Distance() const = 0;

    /// \brief Store the current distance cable-connector.
    public: void Update();

    /// \brief History of tier 2 stats.
    private: std::vector<StatsTier2> allStats;
  };
}
#endif
