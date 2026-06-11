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

#include "aic_scoring/ScoringTier2.hh"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace aic_scoring {
//////////////////////////////////////////////////
ScoringTier2::ScoringTier2(rclcpp::Node *_node,
                           const std::string &_gripperFrame,
                           const CableConnections &_connections)
    : node(_node), gripperFrame(_gripperFrame), connection(_connections) {}

//////////////////////////////////////////////////
bool ScoringTier2::StartRecording(const std::chrono::seconds &_max_task_time) {
  this->tf2_buffer = std::make_unique<tf2::BufferCore>(_max_task_time);

  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->state != State::Idle) {
      RCLCPP_ERROR(this->node->get_logger(), "Scoring system is busy.");
      return false;
    }
    this->state = State::Recording;
  }

  auto shared_self = this->shared_from_this();
  std::weak_ptr<ScoringTier2> weak_self = shared_self;

  this->static_tf_sub = this->node->create_subscription<TFMsg>(
      kTfStaticTopic, rclcpp::QoS(10), [weak_self](const TFMsg::SharedPtr msg) {
        if (auto self = weak_self.lock()) {
          self->TfStaticCallback(*msg);
        }
      });

  this->tf_sub = this->node->create_subscription<TFMsg>(
      kTfTopic, rclcpp::QoS(10), [weak_self](const TFMsg::SharedPtr msg) {
        if (auto self = weak_self.lock()) {
          self->TfCallback(*msg);
        }
      });

  this->scoring_tf_sub = this->node->create_subscription<TFMsg>(
      kScoringTfTopic, rclcpp::QoS(10),
      [weak_self](const TFMsg::SharedPtr msg) {
        if (auto self = weak_self.lock()) {
          self->TfCallback(*msg);
        }
      });

  this->contacts_sub = this->node->create_subscription<ContactsMsg>(
      kContactsTopic, rclcpp::QoS(10),
      [weak_self](const ContactsMsg::SharedPtr msg) {
        if (auto self = weak_self.lock()) {
          self->ContactsCallback(*msg);
        }
      });

  this->wrench_sub = this->node->create_subscription<WrenchMsg>(
      kWrenchTopic, rclcpp::QoS(10),
      [weak_self](const WrenchMsg::SharedPtr msg) {
        if (auto self = weak_self.lock()) {
          self->WrenchCallback(*msg);
        }
      });

  this->insertion_event_sub = this->node->create_subscription<StringMsg>(
      kInsertionEventTopic, rclcpp::QoS(10),
      [weak_self](const StringMsg::SharedPtr msg) {
        if (auto self = weak_self.lock()) {
          self->InsertionEventCallback(*msg);
        }
      });

  this->cable_activated_sub = this->node->create_subscription<StringMsg>(
      kCableActivatedTopic, rclcpp::QoS(10),
      [weak_self](const StringMsg::SharedPtr msg) {
        if (auto self = weak_self.lock()) {
          self->CableActivatedCallback(*msg);
        }
      });

  this->controller_state_sub =
      this->node->create_subscription<ControllerStateMsg>(
          kControllerStateTopic, rclcpp::QoS(10),
          [weak_self](const ControllerStateMsg::SharedPtr msg) {
            if (auto self = weak_self.lock()) {
              self->ControllerStateCallback(*msg);
            }
          });

  return this->WaitForTfs();
}

//////////////////////////////////////////////////
bool ScoringTier2::WaitForTfs() {
  // Simple spinlock to avoid locking, condition variables etc. for a fairly
  // straightforward wait.
  if (!this->connection.has_value()) {
    RCLCPP_ERROR(this->node->get_logger(),
                 "No connection specified when waiting for tfs");
    return false;
  }
  const auto start = this->node->get_clock()->now();
  const auto timeout = std::chrono::seconds(30);
  while (rclcpp::ok() &&
         (!this->GetTransform(tf2::TimePointZero, "cable_0", "default", true) ||
          !this->GetTransform(tf2::TimePointZero, "cable_1", "default", true) ||
          !this->GetTransform(tf2::TimePointZero, "cable_2", "default", true) ||
          !this->GetTransform(tf2::TimePointZero, "cable_3", "default", true) ||
          !this->GetTransform(tf2::TimePointZero, "cable_4", "default", true) ||
          !this->GetTransform(tf2::TimePointZero, this->gripperFrame, "root",
                              true) ||
          !this->GetTransform(tf2::TimePointZero,
                              this->connection.value().data[0].PortTfName(),
                              "default", true) ||
          !this->GetTransform(tf2::TimePointZero,
                              this->connection.value().data[1].PortTfName(),
                              "default", true)) &&
         this->node->get_clock()->now() - start < timeout) {
    this->node->get_clock()->sleep_for(
        rclcpp::Duration(std::chrono::milliseconds(100)));
  }
  if (this->node->get_clock()->now() - start > timeout) {
    RCLCPP_ERROR(this->node->get_logger(),
                 "Timeout while waiting for transforms for scoring.");
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
bool ScoringTier2::StopRecording() {
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->state != State::Recording) {
    RCLCPP_ERROR(this->node->get_logger(), "Scoring system is not recording");
    return false;
  }
  this->state = State::Idle;
  // Clear all the subscriptions
  this->static_tf_sub.reset();
  this->tf_sub.reset();
  this->scoring_tf_sub.reset();
  this->contacts_sub.reset();
  this->wrench_sub.reset();
  this->insertion_event_sub.reset();
  this->cable_activated_sub.reset();
  this->controller_state_sub.reset();
  return true;
}

//////////////////////////////////////////////////
std::pair<Tier2Score, Tier3Score> ScoringTier2::ComputeScore() {
  Tier2Score tier2_score("Scoring failed.");
  if (this->state != State::Idle) {
    RCLCPP_ERROR(this->node->get_logger(), "Scoring system is busy.");
    return {tier2_score, Tier3Score(0, "Task execution failed.")};
  }

  tier2_score.message = "Scoring succeeded.";

  this->state = State::Idle;
  tier2_score.add_category_score("insertion force",
                                 this->GetInsertionForceScore());
  tier2_score.add_category_score("contacts", this->GetContactsScore());
  const auto [success, tier3_score] = this->CombineTier3Score();
  tier2_score.add_category_score("duration",
                                 this->GetTaskDurationScore(success));
  tier2_score.add_category_score("trajectory smoothness",
                                 this->GetTrajectoryJerkScore(success));
  tier2_score.add_category_score("trajectory efficiency",
                                 this->GetTrajectoryEfficiencyScore(success));

  return {tier2_score, tier3_score};
}

//////////////////////////////////////////////////
std::set<std::string> ScoringTier2::GetMissingRequiredTopics() const {
  std::set<std::string> unavailable;
  auto check = [this, &unavailable](const char *topic) {
    if (this->node->count_publishers(topic) == 0) {
      unavailable.insert(topic);
    }
  };

  check(kTfStaticTopic);
  check(kTfTopic);
  check(kScoringTfTopic);
  check(kContactsTopic);
  check(kWrenchTopic);
  check(kInsertionEventTopic);
  check(kCableActivatedTopic);
  check(kControllerStateTopic);

  return unavailable;
}

bool ScoringTier2::IsRecording() const {
  return this->state == State::Recording;
}

//////////////////////////////////////////////////
void ScoringTier2::SetTaskStartTime(const rclcpp::Time &_time) {
  this->task_start_time = _time;
}

//////////////////////////////////////////////////
void ScoringTier2::SetTaskEndTime(const rclcpp::Time &_time) {
  this->task_end_time = _time;
}

//////////////////////////////////////////////////
void ScoringTier2::TfCallback(const TFMsg &_msg) {
  for (const auto &tf : _msg.transforms) {
    this->tf2_buffer->setTransform(tf, "scoring", false);
  }
  // TODO(luca) we should find a way to only push poses if the gripper pose
  // would have changed because of a new TF message, otherwise we might push
  // poses that are just interpolated by the TF library.
  // Right now, it seems the vast majority of messages under /tf are from the
  // robot state publisher so this should be fine.
  auto end_effector_pose = this->EndEffectorPose(tf2::TimePointZero);
  if (end_effector_pose.has_value()) {
    // It seems we can receive multiple different poses with the same timestamp
    // TODO(luca) consider throttling the robot state publisher
    if (!this->endEffectorPoses.empty() &&
        this->endEffectorPoses.back().header.stamp ==
            end_effector_pose.value().header.stamp) {
      return;
    }
    this->endEffectorPoses.push_back(end_effector_pose.value());
  }
}

//////////////////////////////////////////////////
void ScoringTier2::TfStaticCallback(const TFMsg &_msg) {
  for (const auto &tf : _msg.transforms) {
    this->tf2_buffer->setTransform(tf, "scoring", true);
  }
}

//////////////////////////////////////////////////
void ScoringTier2::ContactsCallback(const ContactsMsg &_msg) {
  if (!_msg.contacts.empty()) {
    this->contacts.push_back(_msg);
  }
}

//////////////////////////////////////////////////
void ScoringTier2::WrenchCallback(const WrenchMsg &_msg) {
  const auto time = rclcpp::Time(_msg.header.stamp);
  this->wrenches.push_back({time.seconds(), _msg.wrench.force});
}

//////////////////////////////////////////////////
void ScoringTier2::CableActivatedCallback(const StringMsg &_msg) {
  if (!this->trackedCable.has_value()) {
    this->trackedCable = _msg.data;
    RCLCPP_INFO(this->node->get_logger(), "Detected contact with cable %s",
                _msg.data.c_str());
  } else {
    if (this->trackedCable.value() != _msg.data) {
      // TODO(luca) Consider applying a penalty if another cable
      // is activated in the same scoring run
      RCLCPP_WARN(this->node->get_logger(),
                  "Cable %s was being tracked but a new cable %s was activated,"
                  " this is not allowed and might result in a penalty.",
                  this->trackedCable.value().c_str(), _msg.data.c_str());
    }
  }
}

//////////////////////////////////////////////////
void ScoringTier2::InsertionEventCallback(const StringMsg &_msg) {
  this->insertionPortNamespaces.insert(_msg.data);
}

//////////////////////////////////////////////////
void ScoringTier2::ControllerStateCallback(const ControllerStateMsg &_msg) {
  auto toSeconds = [](const builtin_interfaces::msg::Time &t) {
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
  };

  // Duplicated timestamp check
  const auto stamp = toSeconds(_msg.header.stamp);
  if (this->endEffectorVelocities.size() > 0 &&
      this->endEffectorVelocities.back().first == stamp) {
    return;
  }
  this->endEffectorVelocities.push_back({stamp, _msg.tcp_velocity.linear});
}

//////////////////////////////////////////////////
std::optional<ScoringTier2::TransformStampedMsg> ScoringTier2::GetTransform(
    tf2::TimePoint _t, const std::string &_target_frame,
    const std::string &_reference_frame, bool _suppress_error) const {
  std::string error;
  if (!this->tf2_buffer->canTransform(_reference_frame, _target_frame, _t,
                                      &error)) {
    if (!_suppress_error) {
      RCLCPP_ERROR(
          this->node->get_logger(),
          "Transform between %s and %s not found in the tf tree, error: %s",
          _reference_frame.c_str(), _target_frame.c_str(), error.c_str());
    }
    return std::nullopt;
  }

  return this->tf2_buffer->lookupTransform(_reference_frame, _target_frame, _t);
}

//////////////////////////////////////////////////
std::optional<double> ScoringTier2::GetPlugPortDistance(
    tf2::TimePoint t, std::size_t index) const {
  if (!this->connection.has_value()) {
    RCLCPP_ERROR(this->node->get_logger(), "No connection was found");
    return std::nullopt;
  }
  if (!this->trackedCable.has_value()) {
    RCLCPP_ERROR(this->node->get_logger(), "No cable is active");
    return std::nullopt;
  }
  // For now we only calculate the distance for the first connection
  const auto plug_tf_opt = this->GetTransform(
      t, (*this->connection).data[index].PlugTfName(*this->trackedCable));
  const auto port_tf_opt =
      this->GetTransform(t, (*this->connection).data[index].PortTfName());
  if (!plug_tf_opt.has_value() || !port_tf_opt.has_value()) {
    return std::nullopt;
  }
  const auto plug_tf = plug_tf_opt.value();
  const auto port_tf = port_tf_opt.value();

  return std::sqrt(
      (plug_tf.transform.translation.x - port_tf.transform.translation.x) *
          (plug_tf.transform.translation.x - port_tf.transform.translation.x) +
      (plug_tf.transform.translation.y - port_tf.transform.translation.y) *
          (plug_tf.transform.translation.y - port_tf.transform.translation.y) +
      (plug_tf.transform.translation.z - port_tf.transform.translation.z) *
          (plug_tf.transform.translation.z - port_tf.transform.translation.z));
}

//////////////////////////////////////////////////
// Calculates an inverse proportional score clamped to max_score for min_range
// and min_score for max_range, and with a linear inverse proportional
// interpolation inbetween.
static double CalculateInverseProportionalScore(const double max_score,
                                                const double min_score,
                                                const double max_range,
                                                const double min_range,
                                                const double measurement) {
  if (measurement >= max_range) {
    return min_score;
  } else if (measurement <= min_range) {
    return max_score;
  }

  return min_score + ((max_range - measurement) / (max_range - min_range)) *
                         (max_score - min_score);
}

//////////////////////////////////////////////////
Tier2Score::CategoryScore ScoringTier2::GetTrajectoryJerkScore(
    const bool _success) const {
  using CategoryScore = Tier2Score::CategoryScore;

  const double kMaxJerkScore = 6.0;
  const double kMinJerkScore = 0.0;
  const double kMaxJerkValue = 50.0;
  const double kMinJerkValue = 0.0;

  // Filter parameters, window size in samples. For 500 hz = 30 ms
  static constexpr std::size_t kWindowSize = 15;
  static constexpr std::size_t k = kWindowSize / 2;

  if (!this->task_end_time.has_value()) {
    return CategoryScore(0, "Task not completed.");
  }

  if (!_success) {
    return CategoryScore(
        0,
        "Plug is not within max bounding radius from target port, "
        "not assigning jerk bonus");
  }

  if (this->endEffectorVelocities.size() < kWindowSize) {
    return CategoryScore(0.0,
                         "Insufficient velocity samples for jerk computation.");
  }

  Eigen::MatrixXd A(kWindowSize, 3);
  Eigen::VectorXd y_x(kWindowSize), y_y(kWindowSize), y_z(kWindowSize);

  auto computeJerk = [&](const std::size_t index) {
    const auto &data = this->endEffectorVelocities;
    // Second order (quadratic) polynomial fit to velocity
    // (y = c0 + c1 * dt + c2 * dt^2)

    const double tCenter = data[index].first;

    for (std::size_t j = 0; j < kWindowSize; ++j) {
      const std::size_t dataIdx = index - k + j;
      const double dt = data[dataIdx].first - tCenter;

      // Vandermonde matrix
      A(j, 0) = 1.0;
      A(j, 1) = dt;
      A(j, 2) = dt * dt;
      // Target values of the polynomial function to be found
      y_x(j) = data[dataIdx].second.x;
      y_y(j) = data[dataIdx].second.y;
      y_z(j) = data[dataIdx].second.z;
    }

    // Solve for the polynomial coefficients
    auto qr = A.colPivHouseholderQr();
    Eigen::VectorXd c_x = qr.solve(y_x);
    Eigen::VectorXd c_y = qr.solve(y_y);
    Eigen::VectorXd c_z = qr.solve(y_z);

    // The jerk is the second derivative of the local polynomial approximation,
    // hence 2 * c2
    Vector3Msg ret;
    ret.x = 2.0 * c_x(2);
    ret.y = 2.0 * c_y(2);
    ret.z = 2.0 * c_z(2);
    return ret;
  };

  double totalJerkTime = 0.0;
  double accumLinearJerkMagnitude = 0.0;

  for (std::size_t i = k; i < this->endEffectorVelocities.size() - k; ++i) {
    const auto &v = this->endEffectorVelocities[i].second;
    constexpr double kVelocityThreshold = 0.01;
    // Compute velocity at the central sample to gate jerk accumulation.
    // Only accumulate jerk when the arm is actually moving, so that stillness
    // periods don't dilute the average toward zero.
    const double speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (speed <= kVelocityThreshold) {
      continue;
    }
    const auto j = computeJerk(i);
    const double jerkMag = std::sqrt(j.x * j.x + j.y * j.y + j.z * j.z);
    const double t0 = this->endEffectorVelocities[i - k].first;
    const double t1 = this->endEffectorVelocities[i + k].first;
    const double dt = (t1 - t0) / kWindowSize;
    totalJerkTime += dt;
    accumLinearJerkMagnitude += jerkMag * dt;
  }

  if (std::abs(totalJerkTime) < 1e-6) {
    const std::string msg =
        "Error computing jerk. Insufficient end-effector pose samples.";
    RCLCPP_ERROR(this->node->get_logger(), msg.c_str());
    return CategoryScore(0.0, msg);
  }

  const double jerk = accumLinearJerkMagnitude / totalJerkTime;

  std::stringstream sstream;
  sstream.setf(std::ios::fixed);
  sstream.precision(2);
  sstream << "Average linear jerk magnitude of the end effector: " << jerk
          << " m/s^3";

  const double score = CalculateInverseProportionalScore(
      kMaxJerkScore, kMinJerkScore, kMaxJerkValue, kMinJerkValue, jerk);

  return CategoryScore(score, sstream.str());
}

//////////////////////////////////////////////////
Tier2Score::CategoryScore ScoringTier2::GetTrajectoryEfficiencyScore(
    const bool _success) const {
  using CategoryScore = Tier2Score::CategoryScore;

  if (!this->task_end_time.has_value()) {
    return CategoryScore(0, "Task not completed.");
  }

  if (!_success) {
    return CategoryScore(
        0,
        "Plug is not within max bounding radius from target port, "
        "not assigning efficiency bonus");
  }

  const std::size_t numConnections = this->connection.value().data.size();
  double minPathLength = 0.0;
  // Minimum distance is the sum of all the initialization distances.
  // It should be impossible to reach full score but this is a reasonable
  // minimum distance estimate.
  for (std::size_t index = 0; index < numConnections; ++index) {
    // Compute initial plug-port distance for trajectory efficiency scoring.
    // The robot must travel at least this distance, so it becomes the minimum
    // path length for a perfect score.
    if (this->task_start_time.has_value()) {
      const auto initDist = this->GetPlugPortDistance(
          tf2::TimePoint(std::chrono::nanoseconds(
              this->task_start_time.value().nanoseconds())),
          index);
      if (initDist.has_value()) {
        minPathLength += initDist.value();
      } else {
        RCLCPP_WARN(this->node->get_logger(),
                    "Failed to get initial plug port distance");
      }
    }
  }

  double totalPathLength = 0.0;
  for (std::size_t i = 1; i < this->endEffectorPoses.size(); ++i) {
    const auto &tf0 = this->endEffectorPoses[i - 1].transform.translation;
    const auto &tf1 = this->endEffectorPoses[i].transform.translation;
    double dx = tf1.x - tf0.x;
    double dy = tf1.y - tf0.y;
    double dz = tf1.z - tf0.z;
    totalPathLength += std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  // Score range and path length bounds (meters).
  const double kMaxEfficiencyScore = 6.0;             // Shortest path
  const double kMinEfficiencyScore = 0.0;             // Longest path
  const double kMaxPathLength = 2.0 + minPathLength;  // Path for min score

  std::stringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "Total end-effector path length: " << totalPathLength << " m"
     << ", initial plug-port distance: " << minPathLength << " m";

  const double score = CalculateInverseProportionalScore(
      kMaxEfficiencyScore, kMinEfficiencyScore, kMaxPathLength, minPathLength,
      totalPathLength);

  return CategoryScore(score, ss.str());
}

//////////////////////////////////////////////////
Tier3Score ScoringTier2::GetDistanceScore(std::size_t index) const {
  // A two step approach to compute the score:
  // * If we are in partial insertion, checked through a bounding box between
  //   the port entrance and its end, interpolate linearly in the range.
  // * If we are not in partial insertion, A simple distance metric,
  //   inversely proportional to the time it took to execute the task and the
  //   final distance between plug and port.
  //   Linear interpolation in the interval, clamp to maximum and a bounding
  //   sphere centered in the port tip and with radius between port entrance
  //   and port entrance + maxDistance. The maxDistance is set to half of the
  //   initial plug-port distance.
  //   This score is always lower than a partial insertion score.

  if (!this->task_start_time.has_value()) {
    return Tier3Score(0,
                      "Distance computation failed, task start time not set");
  }

  // Fix the distance to 20 cm
  const double kMaxDistance = 0.2;
  const double kClosestTaskScore = 25.0;
  const double kFurthestTaskScore = 0.0;

  // Starting partial insertion will award kMinInsertionScore, linear range all
  // the way to the end with kMaxInsertionScore.
  const double kMinInsertionScore = 38.0;
  const double kMaxInsertionScore = 50.0;
  // The tolerance in x-y within the port to validate that the plug is being
  // inserted.
  const double kEntranceXYTol = 0.005;

  if (!this->task_end_time.has_value()) {
    return Tier3Score(0, "Task not completed.");
  }

  // We just use the latest sample to avoid race conditions between task end
  // and tf message arrival
  const auto dist = this->GetPlugPortDistance(tf2::TimePointZero, index);
  if (!dist.has_value()) {
    return Tier3Score(
        0, "Distance computation failed, tf between cable and port not found");
  }

  if (!this->trackedCable.has_value()) {
    return Tier3Score(0, "Distance computation failed, no active cable found");
  }

  RCLCPP_INFO(this->node->get_logger(),
              "Final distance between plug and port is %.2f", dist.value());

  // Check if we are in partial insertion
  const auto port_entrance_tf = this->GetTransform(
      tf2::TimePointZero, (*this->connection).data[index].PortEntranceTfName());
  const auto port_tf = this->GetTransform(
      tf2::TimePointZero, (*this->connection).data[index].PortTfName());
  const auto plug_tf = this->GetTransform(
      tf2::TimePointZero,
      (*this->connection).data[index].PlugTfName(this->trackedCable.value()));

  if (!port_entrance_tf.has_value() || !port_tf.has_value() ||
      !plug_tf.has_value()) {
    return Tier3Score(0,
                      "Distance computation failed, tf between plug, port and "
                      "port entrance not found");
  }

  const auto port_entrance_trans =
      port_entrance_tf.value().transform.translation;
  const auto port_trans = port_tf.value().transform.translation;
  const auto plug_trans = plug_tf.value().transform.translation;

  // Used to transition smoothly between the distance scoring and the partial
  // insertion scoring. This is the maximum distance after which no further
  // bonus is given
  const auto distance_threshold =
      std::abs(port_entrance_trans.z - port_trans.z);

  std::stringstream sstream;
  sstream.setf(std::ios::fixed);
  sstream.precision(2);

  // A bounding box with a kEntranceXYTol x-z size, up to port_entrance.z,
  // down until port_trans.z - a small value (for numerical tolerances)
  if (std::abs(plug_trans.x - port_trans.x) < kEntranceXYTol &&
      std::abs(plug_trans.y - port_trans.y) < kEntranceXYTol &&
      plug_trans.z < port_entrance_trans.z &&
      plug_trans.z - port_trans.z > -0.01) {
    // We are in partial insertion, apply a bonus proportional to how far we
    // are from the actual port
    const double port_to_entrance_dist = port_entrance_trans.z - port_trans.z;
    const double plug_to_port_dist = plug_trans.z - port_trans.z;

    // The closest we are the higher we score
    const double score = CalculateInverseProportionalScore(
        kMaxInsertionScore, kMinInsertionScore, port_to_entrance_dist, 0.0,
        plug_to_port_dist);
    sstream << "Partial insertion detected with distance of "
            << plug_to_port_dist << "m.";
    return Tier3Score(score, sstream.str());
  }

  const double score = CalculateInverseProportionalScore(
      kClosestTaskScore, kFurthestTaskScore, distance_threshold + kMaxDistance,
      distance_threshold, dist.value());

  sstream << "No insertion detected. Final plug port distance: " << dist.value()
          << "m.";

  return Tier3Score(score, sstream.str());
}

std::optional<InsertionNamespace> InsertionNamespace::make(std::string msg) {
  InsertionNamespace result;
  std::vector<std::string> tokens;
  size_t pos = 0;
  std::string delimiter = "#";
  while ((pos = msg.find(delimiter)) != std::string::npos) {
    std::string token = msg.substr(0, pos);
    if (!token.empty()) tokens.push_back(token);
    msg.erase(0, pos + delimiter.length());
  }
  tokens.push_back(msg);
  if (tokens.size() != 3) {
    return std::nullopt;
  }
  result.cable_name = tokens[0];
  if (tokens[1] == "0") {
    result.cable_end = 0;
  } else if (tokens[1] == "1") {
    result.cable_end = 1;
  } else {
    return std::nullopt;
  }
  // Now split the namespace with the "/"
  tokens.clear();
  pos = 0;
  delimiter = "/";
  while ((pos = msg.find(delimiter)) != std::string::npos) {
    std::string token = msg.substr(0, pos);
    if (!token.empty()) tokens.push_back(token);
    msg.erase(0, pos + delimiter.length());
  }
  tokens.push_back(msg);
  if (tokens.size() < 2) {
    return std::nullopt;
  }
  if (tokens.size() >= 2u) {
    result.module_name = tokens[0];
    result.port_name = tokens[1];
  }
  return result;
}

//////////////////////////////////////////////////
Tier3Score ScoringTier2::ComputeTier3Score(std::size_t index) const {
  // Binary will award kInsertionCompletionScore, partial insertion computed
  // in GetDistanceScore (and up to kMaxInsertionScore)
  constexpr double kInsertionCompletionScore = 75.0;
  constexpr double kInsertionPenalty = -12.0;

  if (!this->task_end_time.has_value()) {
    return Tier3Score(0, "Task not completed.");
  }

  // Check if insertion is completed or not
  std::stringstream sstream;
  // Intentional copy since we will edit this
  for (auto namespaceStr : this->insertionPortNamespaces) {
    const auto parsedNamespace = InsertionNamespace::make(namespaceStr);
    if (!parsedNamespace.has_value()) {
      RCLCPP_ERROR(this->node->get_logger(), "Failed parsing namespace %s.",
                   namespaceStr.c_str());
    }
    if (!this->trackedCable.has_value() ||
        *this->trackedCable != parsedNamespace->cable_name) {
      RCLCPP_WARN(this->node->get_logger(),
                  "Received insertion event for %s but it is not being "
                  "tracked, ignoring...",
                  parsedNamespace->cable_name.c_str());
      continue;
    }
    if (parsedNamespace->cable_end != index) {
      continue;
    }

    const auto &targetModuleName =
        (*this->connection).data[index].targetModuleName;
    const auto &portName = (*this->connection).data[index].portName;

    if (parsedNamespace->module_name == targetModuleName &&
        parsedNamespace->port_name == portName) {
      return Tier3Score(kInsertionCompletionScore,
                        "Cable insertion successful.");
    } else {
      return Tier3Score(
          kInsertionPenalty,
          std::string("Cable insertion failed. Incorrect Port [") +
              parsedNamespace->module_name + "/" + parsedNamespace->port_name +
              "], while the task requested [" + targetModuleName + "/" +
              portName + "].");
    }
  }
  // Cable insertion was not completed, compute partial insertion
  return this->GetDistanceScore(index);
}

std::pair<bool, Tier3Score> ScoringTier2::CombineTier3Score() const {
  const auto score_0 = this->ComputeTier3Score(0);
  const auto score_1 = this->ComputeTier3Score(1);
  const bool successful =
      score_0.total_score() > 0 && score_1.total_score() > 0;
  const auto combinedScore =
      (score_0.total_score() + score_1.total_score()) / 2.0;
  const std::string msg = "[Task 0] : '" + score_0.message + "'. [Task 1]: '" +
                          score_1.message + "'.";
  return {successful, Tier3Score(combinedScore, msg)};
}

//////////////////////////////////////////////////
std::optional<ScoringTier2::TransformStampedMsg> ScoringTier2::EndEffectorPose(
    tf2::TimePoint t) const {
  // Sanity check.
  if (this->gripperFrame.empty()) {
    RCLCPP_ERROR(this->node->get_logger(),
                 "Unable to compute end effector pose, gripper frame not set");
    return std::nullopt;
  }
  return this->GetTransform(t, this->gripperFrame, "root", true);
}

//////////////////////////////////////////////////
Tier2Score::CategoryScore ScoringTier2::GetInsertionForceScore() const {
  using CategoryScore = Tier2Score::CategoryScore;
  // Apply a fixed penalty if excessive force is detected for more than a
  // certain time
  // The sensor reading is tared at startup so its reading is close to 0N.
  const double kForceThreshold = 20.0;
  const double kDurationThreshold = 1.0;
  const double kPenalty = -12.0;

  double max_force = 0.0;
  double time_above_threshold = 0.0;
  // Start from 1 for easier dt calculation
  for (std::size_t i = 1; i < this->wrenches.size(); ++i) {
    const auto &f = this->wrenches[i].second;
    const double force_mag = std::sqrt(f.x * f.x + f.y * f.y + f.z * f.z);
    if (force_mag > kForceThreshold) {
      time_above_threshold +=
          this->wrenches[i].first - this->wrenches[i - 1].first;
    }
    if (force_mag > max_force) max_force = force_mag;
  }

  std::string msg;
  if (time_above_threshold == 0.0) {
    return CategoryScore(0, "No excessive force detected");
  }

  double score = 0.0;
  std::stringstream sstream;
  sstream.setf(std::ios::fixed);
  sstream.precision(2);
  sstream << "Insertion force above " << kForceThreshold
          << " N, detected for a time of " << time_above_threshold
          << " seconds. Max detected force: " << max_force << "N.";

  if (time_above_threshold > kDurationThreshold) {
    score = kPenalty;
    sstream << " This is above the threshold of " << kDurationThreshold
            << " seconds. Penalty applied.";
  } else {
    sstream << " This is below the threshold of " << kDurationThreshold
            << " seconds. Penalty not applied.";
  }

  return CategoryScore(score, sstream.str());
}

//////////////////////////////////////////////////
Tier2Score::CategoryScore ScoringTier2::GetContactsScore() const {
  using CategoryScore = Tier2Score::CategoryScore;
  // Apply a fixed penalty if any contact was detected.
  const double kPenalty = -24.0;
  if (this->contacts.empty()) {
    return CategoryScore(0, "No contact detected.");
  }

  const auto &contact = this->contacts[0].contacts[0];
  std::stringstream sstream;
  sstream.setf(std::ios::fixed);
  sstream.precision(2);
  sstream << "Contacts detected (only first reported) between entity named ["
          << contact.collision1.name << "] and [" << contact.collision2.name
          << "]. Penalty applied.";
  return CategoryScore(kPenalty, sstream.str());
}

//////////////////////////////////////////////////
Tier2Score::CategoryScore ScoringTier2::GetTaskDurationScore(
    const bool _success) const {
  using CategoryScore = Tier2Score::CategoryScore;

  const rclcpp::Duration kMaxTaskTime = rclcpp::Duration::from_seconds(300.0);
  const rclcpp::Duration kMinTaskTime = rclcpp::Duration::from_seconds(60.0);
  const double kFastestTaskScore = 12.0;
  const double kSlowestTaskScore = 0.0;

  if (!this->task_end_time.has_value()) {
    return CategoryScore(0, "Task not completed.");
  }

  if (!this->task_start_time.has_value()) {
    return CategoryScore(0, "Time computation failed, task start time not set");
  }

  if (!_success) {
    return CategoryScore(
        0,
        "Plug is not within max bounding radius from target port, "
        "not assigning time bonus");
  }

  const rclcpp::Duration task_duration =
      this->task_end_time.value() - this->task_start_time.value();
  const double score = CalculateInverseProportionalScore(
      kFastestTaskScore, kSlowestTaskScore, kMaxTaskTime.seconds(),
      kMinTaskTime.seconds(), task_duration.seconds());

  std::stringstream sstream;
  sstream.setf(std::ios::fixed);
  sstream.precision(2);
  sstream << "Task duration: " << task_duration.seconds() << " seconds.";
  return CategoryScore(score, sstream.str());
}

}  // namespace aic_scoring
