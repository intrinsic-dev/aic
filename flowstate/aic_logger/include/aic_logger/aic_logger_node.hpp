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

#ifndef AIC_LOGGER__AIC_LOGGER_NODE_HPP_
#define AIC_LOGGER__AIC_LOGGER_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "aic_logger_interfaces/srv/start_recording.hpp"
#include "aic_logger_interfaces/srv/stop_recording.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_transport/recorder.hpp"

namespace aic_logger {

class AicLoggerNode : public rclcpp::Node {
 public:
  explicit AicLoggerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~AicLoggerNode() override;

  void Configure(const std::string& output_directory,
                 const std::string& storage_id,
                 const std::vector<std::string>& topics,
                 const std::vector<std::string>& services,
                 const std::vector<std::string>& actions,
                 bool record_all_topics,
                 bool record_all_services,
                 bool record_all_actions);

 private:
  void HandleStartRecording(
      const std::shared_ptr<aic_logger_interfaces::srv::StartRecording::Request> request,
      std::shared_ptr<aic_logger_interfaces::srv::StartRecording::Response> response);

  void HandleStopRecording(
      const std::shared_ptr<aic_logger_interfaces::srv::StopRecording::Request> request,
      std::shared_ptr<aic_logger_interfaces::srv::StopRecording::Response> response);

  std::string output_directory_ = "/tmp";
  std::string storage_id_ = "mcap";

  rosbag2_transport::RecordOptions record_options_;

  std::mutex recording_mutex_;
  std::atomic<bool> is_recording_{false};
  std::string current_bag_path_;

  std::shared_ptr<rosbag2_transport::Recorder> recorder_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> recorder_executor_;
  std::thread recorder_thread_;

  rclcpp::Service<aic_logger_interfaces::srv::StartRecording>::SharedPtr start_recording_service_;
  rclcpp::Service<aic_logger_interfaces::srv::StopRecording>::SharedPtr stop_recording_service_;
};

}  // namespace aic_logger

#endif  // AIC_LOGGER__AIC_LOGGER_NODE_HPP_
