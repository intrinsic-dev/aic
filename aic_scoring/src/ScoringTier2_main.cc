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

#include <yaml-cpp/yaml.h>

#include "aic_scoring/ScoringTier2.hh"

//////////////////////////////////////////////////
int main(int argc, char* argv[]) {
  // Sanity check: There should be one argument.
  if (argc != 2) {
    std::cerr << "Usage: scoring_tier1 <tier1_yaml_file>" << std::endl;
    return -1;
  }

  rclcpp::init(argc, argv);

  std::string configFile = std::string(argv[1]);
  auto scoringTier2 =
      std::make_shared<aic_scoring::ScoringTier2Node>(configFile);

  rclcpp::spin(scoringTier2);
  rclcpp::shutdown();
  return 0;
}
