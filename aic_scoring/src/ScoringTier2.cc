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
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sstream>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace aic_scoring {
//////////////////////////////////////////////////
ScoringTier2::ScoringTier2(rclcpp::Node *_node) : node(_node) {}

//////////////////////////////////////////////////
// TODO(luca) consider having a make function that returns a pointer which is
// nullptr if initialization failed instead.
bool ScoringTier2::Initialize(YAML::Node _config) {
  if (!this->node) {
    std::cerr << "[ScoringTier2]: null ROS node. Aborting." << std::endl;
    return false;
  }
  if (!this->ParseStats(_config)) return false;

  return true;
}

//////////////////////////////////////////////////
void ScoringTier2::ResetConnections(
    const std::vector<Connection> &_connections) {
  this->connections = _connections;

  // Debug output.
  // std::cout << "Connections" << std::endl;
  // for (const Connection &c : this->connections)
  // {
  //   std::cout << "  plug: " << c.plugName << std::endl;
  //   std::cout << "  port: " << c.portName << std::endl;
  //   std::cout << "  Dist: " << c.distance << std::endl;
  // }
}

//////////////////////////////////////////////////
void ScoringTier2::SetGripperFrame(const std::string &_gripperFrame) {
  this->gripperFrame = _gripperFrame;
}

//////////////////////////////////////////////////
bool ScoringTier2::StartRecording(const std::string &_filename,
                                  const std::vector<Connection> &_connections,
                                  const std::chrono::seconds &_max_task_time) {
  this->Reset(_max_task_time);
  this->ResetConnections(_connections);
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->state != State::Idle) {
      RCLCPP_ERROR(this->node->get_logger(), "Scoring system is busy.");
      return false;
    }

    try {
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = _filename;
      this->bagWriter.open(storage_options);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->node->get_logger(), "Failed to open bag: %s",
                   e.what());
      return false;
    }
    this->state = State::Recording;
    this->bagUri = _filename;
  }

  // Subscribe to all topics relevant for scoring.
  for (const auto &topic : this->topics) {
    auto qos = topic.latched
                   ? rclcpp::QoS(rclcpp::KeepLast(100)).transient_local()
                   : rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto sub = this->node->create_generic_subscription(
        topic.name, topic.type, qos,
        [this, topic](std::shared_ptr<const rclcpp::SerializedMessage> msg,
                      const rclcpp::MessageInfo &msg_info) {
          // Bag the data.
          const auto &rmw_info = msg_info.get_rmw_message_info();
          std::lock_guard<std::mutex> lock(this->mutex);
          if (this->state == State::Recording) {
            this->bagWriter.write(msg, topic.name, topic.type,
                                  rmw_info.received_timestamp,
                                  rmw_info.source_timestamp);
            if (topic.name == kScoringTfTopic) {
              // A new cable transform was received
              this->cableTfReceived = true;
            } else if (topic.name == kTfTopic) {
              // A new gripper transform was received
              this->gripperTfReceived = true;
            }
          }
        });
    this->subscriptions.push_back(sub);
  }

  return this->WaitForTfs();
}

//////////////////////////////////////////////////
bool ScoringTier2::WaitForTfs() {
  this->cableTfReceived = false;
  this->gripperTfReceived = false;
  // Simple spinlock to avoid locking, condition variables etc. for a fairly
  // straightforward wait.
  const auto start = this->node->get_clock()->now();
  const auto timeout = std::chrono::seconds(10);
  while (rclcpp::ok() && (!this->cableTfReceived || !this->gripperTfReceived) &&
         this->node->get_clock()->now() - start < timeout) {
    this->node->get_clock()->sleep_for(
        rclcpp::Duration(std::chrono::milliseconds(100)));
  }
  if (!this->cableTfReceived || !this->gripperTfReceived) {
    RCLCPP_ERROR(this->node->get_logger(),
                 "Timeout while waiting for transforms for scoring.");
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
bool ScoringTier2::StopRecording() {
  if (!this->WaitForTfs()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->state != State::Recording) {
    RCLCPP_ERROR(this->node->get_logger(), "Scoring system is not recording");
    return false;
  }
  this->bagWriter.close();
  this->state = State::Idle;
  return true;
}

//////////////////////////////////////////////////
template <typename Msg>
Msg deserialize_from_rosbag(
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg_in) {
  Msg msg;
  rclcpp::SerializedMessage extracted_serialized_msg(*msg_in->serialized_data);
  rclcpp::Serialization<Msg> serialization;
  serialization.deserialize_message(&extracted_serialized_msg, &msg);
  return msg;
}

//////////////////////////////////////////////////
std::pair<Tier2Score, Tier3Score> ScoringTier2::ComputeScore() {
  Tier2Score tier2_score("Scoring failed.");
  Tier3Score tier3_score(0, "Task execution failed.");
  if (this->state != State::Idle) {
    RCLCPP_ERROR(this->node->get_logger(), "Scoring system is busy.");
    return {tier2_score, tier3_score};
  }
  rosbag2_cpp::Reader bagReader;

  try {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = this->bagUri;
    bagReader.open(storage_options);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->node->get_logger(), "Failed to open bag: %s", e.what());
    return {tier2_score, tier3_score};
  }
  this->state = State::Scoring;

  tier2_score.message = "Scoring succeeded.";

  // First pass: Process all messages to build the complete TF buffer.
  // We need both static TF (robot URDF) and dynamic TF (joint states) to
  // compute the full transform chain to the gripper.
  while (rclcpp::ok() && bagReader.has_next()) {
    const auto msg_ptr = bagReader.read_next();
    // Debugging to make sure messages are in the bag
    // RCLCPP_INFO(this->node->get_logger(), "Received message on topic '%s'",
    //     msg_ptr->topic_name.c_str());
    if (msg_ptr->topic_name == kJointStateTopic) {
      const auto msg = deserialize_from_rosbag<JointStateMsg>(msg_ptr);
      this->JointStateCallback(msg);
    } else if (msg_ptr->topic_name == kTfTopic ||
               msg_ptr->topic_name == kScoringTfTopic) {
      const auto msg = deserialize_from_rosbag<TFMsg>(msg_ptr);
      this->TfCallback(msg);
    } else if (msg_ptr->topic_name == kTfStaticTopic) {
      const auto msg = deserialize_from_rosbag<TFMsg>(msg_ptr);
      this->TfStaticCallback(msg);
    } else if (msg_ptr->topic_name == kContactsTopic) {
      const auto msg = deserialize_from_rosbag<ContactsMsg>(msg_ptr);
      this->ContactsCallback(msg);
    } else if (msg_ptr->topic_name == kWrenchTopic) {
      const auto msg = deserialize_from_rosbag<WrenchMsg>(msg_ptr);
      this->WrenchCallback(msg);
    } else if (msg_ptr->topic_name == kMotionUpdateTopic) {
      const auto msg = deserialize_from_rosbag<MotionUpdateMsg>(msg_ptr);
      this->MotionUpdateCallback(msg);
    } else if (msg_ptr->topic_name == kJointMotionUpdateTopic) {
      const auto msg = deserialize_from_rosbag<JointMotionUpdateMsg>(msg_ptr);
      this->JointMotionUpdateCallback(msg);
    } else if (msg_ptr->topic_name == kInsertionEventTopic) {
      const auto msg = deserialize_from_rosbag<StringMsg>(msg_ptr);
      this->InsertionEventCallback(msg);
    } else if (msg_ptr->topic_name == kControllerStateTopic) {
      const auto msg = deserialize_from_rosbag<ControllerStateMsg>(msg_ptr);
      this->ControllerStateCallback(msg);
    } else {
      RCLCPP_WARN(this->node->get_logger(),
                  "Unexpected topic name while scoring: %s",
                  msg_ptr->topic_name.c_str());
    }
  }

  // Complete the TF tree by linking world and aic_world
  // The aic_gz_bringup launch file uses a static tf broadcaster to do this
  // when ground truth is enabled. Here we manually add the fixed transform to
  // the tf buffer
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = "world";
  transform_stamped.child_frame_id = "aic_world";
  TFMsg msg;
  msg.transforms.push_back(transform_stamped);
  this->TfStaticCallback(msg);

  this->state = State::Idle;
  // Compute initial plug-port distance for trajectory efficiency scoring.
  // The robot must travel at least this distance, so it becomes the minimum
  // path length for a perfect score.
  double minPathLength = 0.0;
  if (this->task_start_time.has_value()) {
    const auto initDist = this->GetPlugPortDistance(tf2::TimePoint(
        std::chrono::nanoseconds(this->task_start_time.value().nanoseconds())));
    if (initDist.has_value()) {
      minPathLength = initDist.value();
    } else {
      RCLCPP_WARN(this->node->get_logger(),
                  "Failed to get initial plug port distance");
    }
  }
  tier2_score.add_category_score("insertion force",
                                 this->GetInsertionForceScore());
  tier2_score.add_category_score("contacts", this->GetContactsScore());
  tier3_score = this->ComputeTier3Score();
  tier2_score.add_category_score("duration",
                                 this->GetTaskDurationScore(tier3_score));
  tier2_score.add_category_score("trajectory smoothness",
                                 this->GetTrajectoryJerkScore(tier3_score));
  tier2_score.add_category_score(
      "trajectory efficiency",
      this->GetTrajectoryEfficiencyScore(minPathLength, tier3_score));
  return {tier2_score, tier3_score};
}

//////////////////////////////////////////////////
void ScoringTier2::Reset(const std::chrono::seconds &_buffer_size) {
  this->connections.clear();
  this->bagUri.clear();
  this->tf2_buffer = std::make_unique<tf2::BufferCore>(_buffer_size);
  this->state = State::Idle;
  this->wrenches.clear();
  this->endEffectorPoses.clear();
  this->endEffectorVelocities.clear();
  this->task_start_time.reset();
  this->task_end_time.reset();
  this->bagWriter.close();
  this->contacts.clear();
  this->insertionPortNamespace.clear();
  this->lastTaredFt.reset();
}

//////////////////////////////////////////////////
bool ScoringTier2::ParseStats(YAML::Node _config) {
  // Parse topics to subscribe to.
  if (!_config["topics"]) {
    RCLCPP_ERROR(this->node->get_logger(),
                 "Unable to find [topics] in yaml file");
    return false;
  }

  const auto &topics = _config["topics"];
  if (!topics.IsSequence()) {
    RCLCPP_ERROR(this->node->get_logger(),
                 "Unable to find sequence of topics within [topics]");
    return false;
  }

  for (const auto &newTopic : topics) {
    if (!newTopic["topic"]) {
      RCLCPP_ERROR(this->node->get_logger(),
                   "Unrecognized element. It should be [topic]");
      return false;
    }

    const auto &topicProperties = newTopic["topic"];
    if (!topicProperties.IsMap()) {
      RCLCPP_ERROR(this->node->get_logger(),
                   "Unable to find properties within [topic]");
      return false;
    }

    TopicInfo topicInfo;

    if (!topicProperties["name"]) {
      RCLCPP_ERROR(this->node->get_logger(),
                   "Unable to find [name] within [topic]");
      return false;
    }
    topicInfo.name = topicProperties["name"].as<std::string>();

    if (!topicProperties["type"]) {
      RCLCPP_ERROR(this->node->get_logger(),
                   "Unable to find [type] within [topic]");
      return false;
    }
    topicInfo.type = topicProperties["type"].as<std::string>();

    if (topicProperties["latched"]) {
      topicInfo.latched = topicProperties["latched"].as<bool>();
    }

    this->topics.push_back(topicInfo);
  }

  return true;
}

//////////////////////////////////////////////////
std::set<std::string> ScoringTier2::GetMissingRequiredTopics() const {
  std::set<std::string> unavailable;
  for (const auto &subscription : this->subscriptions) {
    if (subscription->get_publisher_count() == 0) {
      unavailable.insert(subscription->get_topic_name());
    }
  }
  return unavailable;
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
void ScoringTier2::JointStateCallback(const JointStateMsg &_msg) { (void)_msg; }

//////////////////////////////////////////////////
void ScoringTier2::TfCallback(const TFMsg &_msg) {
  for (const auto &tf : _msg.transforms) {
    // The task board TFs are bridged to non-static /tf topic so that it
    // does not flood the /tf_static topic. Added workaround here to ensure
    // that these task board related TFs are added to the tree as static TFs
    // otherwise lookup could fail.
    // \todo(iche033) This is a quick patch to resolve task board TF lookup
    // failures. Consider implementing a proper fix that avoids checking every
    // TF.
    if (tf.child_frame_id.find("task_board") != std::string::npos) {
      this->tf2_buffer->setTransform(tf, "scoring", true);
    } else {
      this->tf2_buffer->setTransform(tf, "scoring", false);
    }
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
  // We don't log for else statement since skipping a few wrench messages
  // at startup is not a big issue.
  if (this->lastTaredFt.has_value()) {
    const auto time = rclcpp::Time(_msg.header.stamp);
    Vector3Msg wrench;
    wrench.x = _msg.wrench.force.x - this->lastTaredFt.value().wrench.force.x;
    wrench.y = _msg.wrench.force.y - this->lastTaredFt.value().wrench.force.y;
    wrench.z = _msg.wrench.force.z - this->lastTaredFt.value().wrench.force.z;
    this->wrenches.push_back({time.seconds(), wrench});
  }
}

//////////////////////////////////////////////////
void ScoringTier2::MotionUpdateCallback(const MotionUpdateMsg &_msg) {
  (void)_msg;
}

//////////////////////////////////////////////////
void ScoringTier2::JointMotionUpdateCallback(const JointMotionUpdateMsg &_msg) {
  (void)_msg;
}

//////////////////////////////////////////////////
void ScoringTier2::InsertionEventCallback(const StringMsg &_msg) {
  // \todo(iche033) For now, assume only one insertion event per task
  // Mark insertion completion as true as soon as one insertion is done.
  this->insertionPortNamespace = _msg.data;
}

//////////////////////////////////////////////////
void ScoringTier2::ControllerStateCallback(const ControllerStateMsg &_msg) {
  auto toSeconds = [](const builtin_interfaces::msg::Time &t) {
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
  };

  this->lastTaredFt = _msg.fts_tare_offset;
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
    tf2::TimePoint t) const {
  if (this->connections.empty()) {
    RCLCPP_ERROR(this->node->get_logger(), "No connection was found");
    return std::nullopt;
  }
  // For now we only calculate the distance for the first connection
  const auto plug_tf_opt =
      this->GetTransform(t, this->connections[0].PlugTfName());
  const auto port_tf_opt =
      this->GetTransform(t, this->connections[0].PortTfName());
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
    const Tier3Score &_tier3) const {
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

  if (_tier3.total_score() <= 0) {
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
    double _minPathLength, const Tier3Score &_tier3) const {
  using CategoryScore = Tier2Score::CategoryScore;

  if (!this->task_end_time.has_value()) {
    return CategoryScore(0, "Task not completed.");
  }

  if (_tier3.total_score() <= 0) {
    return CategoryScore(
        0,
        "Plug is not within max bounding radius from target port, "
        "not assigning efficiency bonus");
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
  const double kMaxEfficiencyScore = 6.0;              // Shortest path
  const double kMinEfficiencyScore = 0.0;              // Longest path
  const double kMaxPathLength = 1.0 + _minPathLength;  // Path for min score

  std::stringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "Total end-effector path length: " << totalPathLength << " m"
     << ", initial plug-port distance: " << _minPathLength << " m";

  const double score = CalculateInverseProportionalScore(
      kMaxEfficiencyScore, kMinEfficiencyScore, kMaxPathLength, _minPathLength,
      totalPathLength);

  return CategoryScore(score, ss.str());
}

//////////////////////////////////////////////////
Tier3Score ScoringTier2::GetDistanceScore() const {
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

  // Being as close as possible to the port entrance will award
  // kClosestTaskScore
  const auto initDist = this->GetPlugPortDistance(tf2::TimePoint(
      std::chrono::nanoseconds(this->task_start_time.value().nanoseconds())));
  double radiusFromPort = 0.0;
  if (initDist.has_value()) {
    radiusFromPort = initDist.value() * 0.5;
  } else {
    RCLCPP_WARN(this->node->get_logger(),
                "Failed to get initial plug port distance");
  }

  const double kMaxDistance = radiusFromPort;
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

  const auto end_time =
      std::chrono::nanoseconds(this->task_end_time.value().nanoseconds());
  const auto dist = this->GetPlugPortDistance(tf2::TimePoint(end_time));
  if (!dist.has_value()) {
    return Tier3Score(
        0, "Distance computation failed, tf between cable and port not found");
  }

  // Check if we are in partial insertion
  const auto port_entrance_tf = this->GetTransform(
      tf2::TimePoint(end_time), this->connections[0].PortEntranceTfName());
  const auto port_tf = this->GetTransform(tf2::TimePoint(end_time),
                                          this->connections[0].PortTfName());
  const auto plug_tf = this->GetTransform(tf2::TimePoint(end_time),
                                          this->connections[0].PlugTfName());

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

//////////////////////////////////////////////////
Tier3Score ScoringTier2::ComputeTier3Score() const {
  // Binary will award kInsertionCompletionScore, partial insertion computed
  // in GetDistanceScore (and up to kMaxInsertionScore)
  constexpr double kInsertionCompletionScore = 75.0;
  constexpr double kInsertionPenalty = -12.0;

  if (!this->task_end_time.has_value()) {
    return Tier3Score(0, "Task not completed.");
  }

  // Check if insertion is completed or not
  std::stringstream sstream;
  if (!this->insertionPortNamespace.empty()) {
    // Tokenize the namespace string. The first token should be the
    // target module name and the second token should be the port name
    std::string namespaceStr = this->insertionPortNamespace;
    std::vector<std::string> tokens;
    size_t pos = 0;
    std::string token;
    std::string delimiter = "/";
    while ((pos = namespaceStr.find(delimiter)) != std::string::npos) {
      std::string token = namespaceStr.substr(0, pos);
      if (!token.empty()) tokens.push_back(token);
      namespaceStr.erase(0, pos + delimiter.length());
    }
    tokens.push_back(namespaceStr);

    if (tokens.size() >= 2u) {
      // Verify the plug is inserted into the correct target port
      if (tokens[0] == connections[0].targetModuleName &&
          tokens[1] == connections[0].portName) {
        return Tier3Score(kInsertionCompletionScore,
                          "Cable insertion successful.");
      } else {
        return Tier3Score(kInsertionPenalty,
                          "Cable insertion failed. Incorrect Port.");
      }
    } else {
      const std::string msg = "Error parsing insertion port namespace: " +
                              this->insertionPortNamespace;
      RCLCPP_ERROR(this->node->get_logger(), msg.c_str());
      return Tier3Score(0.0, msg);
    }
  }
  // Cable insertion was not completed, compute partial insertion
  return this->GetDistanceScore();
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
  return this->GetTransform(t, this->gripperFrame, "aic_world", true);
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
    const Tier3Score &_tier3) const {
  using CategoryScore = Tier2Score::CategoryScore;

  const rclcpp::Duration kMaxTaskTime = rclcpp::Duration::from_seconds(60.0);
  const rclcpp::Duration kMinTaskTime = rclcpp::Duration::from_seconds(5.0);
  const double kFastestTaskScore = 12.0;
  const double kSlowestTaskScore = 0.0;

  if (!this->task_end_time.has_value()) {
    return CategoryScore(0, "Task not completed.");
  }

  if (!this->task_start_time.has_value()) {
    return CategoryScore(0, "Time computation failed, task start time not set");
  }

  if (_tier3.total_score() <= 0) {
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

//////////////////////////////////////////////////
ScoringTier2Node::ScoringTier2Node(const std::string &_yamlFile)
    : Node("score_tier2_node") {
  try {
    auto config = YAML::LoadFile(_yamlFile);
    this->score = std::make_unique<aic_scoring::ScoringTier2>(this);
    this->score->Initialize(config);
  } catch (const YAML::BadFile &_e) {
    std::cerr << "Unable to open YAML file [" << _yamlFile << "]" << std::endl;
    return;
  }
}

}  // namespace aic_scoring
