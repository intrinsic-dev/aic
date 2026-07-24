// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "aic_logger/aic_logger_node.hpp"

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"

namespace aic_logger {

namespace fs = std::filesystem;

static std::string GetCurrentTimeString() {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
  return ss.str();
}

AicLoggerNode::AicLoggerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("aic_logger", options) {
  start_recording_service_ = create_service<aic_logger_interfaces::srv::StartRecording>(
      "~/start_recording",
      std::bind(&AicLoggerNode::HandleStartRecording, this,
                std::placeholders::_1, std::placeholders::_2));

  stop_recording_service_ = create_service<aic_logger_interfaces::srv::StopRecording>(
      "~/stop_recording",
      std::bind(&AicLoggerNode::HandleStopRecording, this,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "AicLoggerNode initialized.");
}

AicLoggerNode::~AicLoggerNode() {
  std::lock_guard<std::mutex> lock(recording_mutex_);
  if (is_recording_) {
    if (recorder_) {
      recorder_->stop();
    }
    if (recorder_executor_) {
      recorder_executor_->cancel();
    }
    if (recorder_thread_.joinable()) {
      recorder_thread_.join();
    }
  }
}

void AicLoggerNode::Configure(const std::string& output_directory,
                             const std::string& storage_id,
                             const std::vector<std::string>& topics,
                             const std::vector<std::string>& services,
                             const std::vector<std::string>& actions,
                             bool record_all_topics,
                             bool record_all_services,
                             bool record_all_actions) {
  output_directory_ = output_directory.empty() ? "/tmp" : output_directory;
  storage_id_ = storage_id.empty() ? "mcap" : storage_id;

  record_options_.topics = topics;
  record_options_.services = services;
  record_options_.actions = actions;
  record_options_.all_topics = record_all_topics;
  record_options_.all_services = record_all_services;
  record_options_.all_actions = record_all_actions;
  record_options_.is_discovery_disabled = false;

  RCLCPP_INFO(get_logger(),
              "Configured aic_logger (output_dir: %s, storage_id: %s, "
              "all_topics: %d, all_services: %d, all_actions: %d)",
              output_directory_.c_str(), storage_id_.c_str(),
              record_all_topics, record_all_services, record_all_actions);
}

void AicLoggerNode::HandleStartRecording(
    const std::shared_ptr<aic_logger_interfaces::srv::StartRecording::Request> request,
    std::shared_ptr<aic_logger_interfaces::srv::StartRecording::Response> response) {
  std::lock_guard<std::mutex> lock(recording_mutex_);

  if (is_recording_) {
    response->success = false;
    response->message = "Recording is already active.";
    response->bag_path = current_bag_path_;
    RCLCPP_WARN(get_logger(), "StartRecording requested but recording is already active.");
    return;
  }

  std::string name_suffix = request->name;
  if (name_suffix.empty()) {
    name_suffix = "recording";
  }

  std::string datetime_str = GetCurrentTimeString();
  std::string bag_name = datetime_str + "_" + name_suffix;
  fs::path full_bag_path = fs::path(output_directory_) / bag_name;
  std::string bag_path_str = full_bag_path.string();

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path_str;
  storage_options.storage_id = storage_id_;

  try {
    auto writer = std::make_shared<rosbag2_cpp::Writer>();
    recorder_ = std::make_shared<rosbag2_transport::Recorder>(
        writer, storage_options, record_options_, "aic_rosbag2_recorder");

    recorder_->record();

    recorder_executor_ =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    recorder_executor_->add_node(recorder_);

    recorder_thread_ = std::thread([this]() {
      if (recorder_executor_) {
        recorder_executor_->spin();
      }
    });

    is_recording_ = true;
    current_bag_path_ = bag_path_str;

    response->success = true;
    response->message = "Recording started successfully.";
    response->bag_path = bag_path_str;

    RCLCPP_INFO(get_logger(), "Started recording to '%s'", bag_path_str.c_str());
  } catch (const std::exception& ex) {
    response->success = false;
    response->message = std::string("Failed to start recording: ") + ex.what();
    response->bag_path = "";
    RCLCPP_ERROR(get_logger(), "Failed to start recording: %s", ex.what());
  }
}

void AicLoggerNode::HandleStopRecording(
    const std::shared_ptr<aic_logger_interfaces::srv::StopRecording::Request> /*request*/,
    std::shared_ptr<aic_logger_interfaces::srv::StopRecording::Response> response) {
  std::lock_guard<std::mutex> lock(recording_mutex_);

  if (!is_recording_) {
    response->success = false;
    response->message = "No active recording to stop.";
    response->bag_path = "";
    RCLCPP_WARN(get_logger(), "StopRecording requested but no recording is active.");
    return;
  }

  try {
    if (recorder_) {
      recorder_->stop();
    }
    if (recorder_executor_) {
      recorder_executor_->cancel();
    }
    if (recorder_thread_.joinable()) {
      recorder_thread_.join();
    }
    if (recorder_executor_ && recorder_) {
      recorder_executor_->remove_node(recorder_);
    }

    recorder_.reset();
    recorder_executor_.reset();

    std::string stopped_bag_path = current_bag_path_;
    is_recording_ = false;

    response->success = true;
    response->message = "Recording stopped successfully.";
    response->bag_path = stopped_bag_path;

    RCLCPP_INFO(get_logger(), "Stopped recording bag '%s'", stopped_bag_path.c_str());
  } catch (const std::exception& ex) {
    response->success = false;
    response->message = std::string("Error stopping recording: ") + ex.what();
    response->bag_path = current_bag_path_;
    RCLCPP_ERROR(get_logger(), "Error stopping recording: %s", ex.what());
  }
}

}  // namespace aic_logger
