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

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <string>
#include <vector>

namespace aic_scoring {
//////////////////////////////////////////////////
ScoringTier2::ScoringTier2(rclcpp::Node *_node) : node(_node) {}

//////////////////////////////////////////////////
// TODO(luca) consider having a make function that returns a pointer which is
// nullptr if initialization failed instead.
bool ScoringTier2::Initialize() {
  if (!this->node) {
    std::cerr << "[ScoringTier2]: null ROS node. Aborting." << std::endl;
    return false;
  }

  const rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  // Subscribe to all topics relevant for scoring.
  this->jointStateSub = this->node->create_subscription<sensor_msgs::msg::JointState>(
    kJointStateTopic,
    reliable_qos,
    [this](std::shared_ptr<const rclcpp::SerializedMessage> msg, const rclcpp::MessageInfo &msg_info) {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->bagOpen) {
        const auto &rmw_info = msg_info.get_rmw_message_info();
        this->bagWriter.write(msg, kJointStateTopic, rosidl_generator_traits::name<sensor_msgs::msg::JointState>(),
                              rmw_info.received_timestamp,
                              rmw_info.source_timestamp);
      }
    }
  );

  this->tfSub = this->node->create_subscription<tf2_msgs::msg::TFMessage>(
    kTfTopic,
    reliable_qos,
    [this](std::shared_ptr<const rclcpp::SerializedMessage> msg, const rclcpp::MessageInfo &msg_info) {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->bagOpen) {
        const auto &rmw_info = msg_info.get_rmw_message_info();
        this->bagWriter.write(msg, kTfTopic, rosidl_generator_traits::name<tf2_msgs::msg::TFMessage>(),
                              rmw_info.received_timestamp,
                              rmw_info.source_timestamp);
      }
    }
  );

  this->tfStaticSub = this->node->create_subscription<tf2_msgs::msg::TFMessage>(
    kTfStaticTopic,
    reliable_qos,
    [this](std::shared_ptr<const rclcpp::SerializedMessage> msg, const rclcpp::MessageInfo &msg_info) {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->bagOpen) {
        const auto &rmw_info = msg_info.get_rmw_message_info();
        this->bagWriter.write(msg, kTfStaticTopic, rosidl_generator_traits::name<tf2_msgs::msg::TFMessage>(),
                              rmw_info.received_timestamp,
                              rmw_info.source_timestamp);
      }
    }
  );

  // TODO(luca) reliable qos
  this->contactsSub = this->node->create_subscription<ros_gz_interfaces::msg::Contacts>(
    kContactsTopic,
    reliable_qos,
    [this](std::shared_ptr<const rclcpp::SerializedMessage> msg, const rclcpp::MessageInfo &msg_info) {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->bagOpen) {
        const auto &rmw_info = msg_info.get_rmw_message_info();
        this->bagWriter.write(msg, kContactsTopic, rosidl_generator_traits::name<ros_gz_interfaces::msg::Contacts>(),
                              rmw_info.received_timestamp,
                              rmw_info.source_timestamp);
      }
    }
  );

  this->wrenchSub = this->node->create_subscription<geometry_msgs::msg::WrenchStamped>(
    kWrenchTopic,
    reliable_qos,
    [this](std::shared_ptr<const rclcpp::SerializedMessage> msg, const rclcpp::MessageInfo &msg_info) {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->bagOpen) {
        const auto &rmw_info = msg_info.get_rmw_message_info();
        this->bagWriter.write(msg, kWrenchTopic, rosidl_generator_traits::name<geometry_msgs::msg::WrenchStamped>(),
                              rmw_info.received_timestamp,
                              rmw_info.source_timestamp);
      }
    }
  );

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
bool ScoringTier2::StartRecording(const std::string &_filename) {
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->bagOpen) {
    RCLCPP_ERROR(this->node->get_logger(), "Bag already opened.");
    return false;
  }

  try {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = _filename;
    this->bagWriter.open(storage_options);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->node->get_logger(), "Failed to open bag: %s", e.what());
    return false;
  }
  this->bagOpen = true;
  return true;
}

//////////////////////////////////////////////////
bool ScoringTier2::StopRecording() {
  std::lock_guard<std::mutex> lock(this->mutex);
  if (!this->bagOpen) {
    RCLCPP_ERROR(this->node->get_logger(), "Bag already closed.");
    return false;
  }
  this->bagWriter.close();
  this->bagOpen = false;
  return true;
}

//////////////////////////////////////////////////
std::set<std::string> ScoringTier2::GetMissingRequiredTopics() const {
  std::set<std::string> unavailable;
  if (this->wrenchSub->get_publisher_count() == 0) {
    unavailable.insert(this->wrenchSub->get_topic_name());
  }
  if (this->jointStateSub->get_publisher_count() == 0) {
    unavailable.insert(this->jointStateSub->get_topic_name());
  }
  return unavailable;
}

//////////////////////////////////////////////////
ScoringTier2Node::ScoringTier2Node(const std::string &_yamlFile)
    : Node("score_tier2_node") {
  try {
    auto config = YAML::LoadFile(_yamlFile);
    this->score = std::make_unique<aic_scoring::ScoringTier2>(this);
    this->score->Initialize();
  } catch (const YAML::BadFile &_e) {
    std::cerr << "Unable to open YAML file [" << _yamlFile << "]" << std::endl;
    return;
  }
}
}  // namespace aic_scoring
