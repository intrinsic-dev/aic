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

#ifndef AIC_SCORING__TIER_SCORE_HH_
#define AIC_SCORING__TIER_SCORE_HH_

#include <map>
#include <optional>
#include <string>

#include "yaml-cpp/yaml.h"

namespace aic_scoring {


class TierScore {
protected:
  TierScore(const std::string& msg = "") : message(msg) {}

public:
  virtual ~TierScore() = default;

  std::string message;

  virtual double total_score() const = 0;

  virtual YAML::Node to_yaml() const {
    YAML::Node score;
    score["score"] = this->total_score();
    score["message"] = this->message;
    return score;
  }
};

class Tier1Score : public TierScore {
private:
  // Score for successful model validation (binary)
  static const int kTier1Success = 1;

  int score;

public:
  Tier1Score() : Tier1Score(false) { }
  Tier1Score(bool success) {
    if (success) {
      this->score = kTier1Success;
      this->message = "Model validation succeeded.";
    } else {
      this->score = 0;
      this->message = "Model validation failed.";
    }
  }

  double total_score() const override {
    return score;
  }
};


class Tier2Score : public TierScore {
public:
  struct CategoryScore {
    double score;
    std::optional<std::string> message;

    CategoryScore(double s, const std::optional<std::string>& msg) : score(s), message(msg) {}
  };

  using CategoryScores = std::map<std::string, CategoryScore>;

  Tier2Score(const std::string& msg) : TierScore(msg) {}

  double total_score() const override {
    double score = 0;
    for (const auto& category : category_scores) {
      score += category.second.score;
    }
    return score;
  }

  YAML::Node to_yaml() const override {
    YAML::Node score;
    score["score"] = this->total_score();
    score["message"] = this->message;
    for (const auto& [name, category_score] : this->category_scores) {
      score["categories"][name]["score"] = category_score.score;
      if (category_score.message.has_value()) {
        score["categories"][name]["message"] = category_score.message.value();
      }
    }
    return score;
  }

  void add_category_score(const std::string& category, double score,
                          const std::optional<std::string>& msg = std::nullopt) {
    this->category_scores.insert({category, CategoryScore(score, msg)});
  }

  void add_category_score(const std::string& category,
                          const CategoryScore& score) {
    this->category_scores.insert({category, score});
  }

private:
  // Map of category name to its score
  CategoryScores  category_scores;

};

class Tier3Score : public TierScore {
private:
  double score;

public:
  Tier3Score(double s, const std::string& msg) : TierScore(msg), score(s) { }

  double total_score() const override {
    return score;
  }
};

} // namespace aic_scoring

#endif
