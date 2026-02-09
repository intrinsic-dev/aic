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

#ifndef AIC_ADAPTER_HPP_
#define AIC_ADAPTER_HPP_

#include <cstdlib>
#include <cstring>
#include <deque>
#include <format>
#include <memory>

#include "aic_control_interfaces/msg/controller_state.hpp"
#include "aic_model_interfaces/msg/observation.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace aic {

class AicAdapterNode : public rclcpp::Node {
 public:
  AicAdapterNode();
  virtual ~AicAdapterNode() {}

 private:
  void image_callback(size_t camera_idx,
                      sensor_msgs::msg::Image::UniquePtr msg);
  void ReorderJointState(const sensor_msgs::msg::JointState& original,
                         sensor_msgs::msg::JointState& reordered);

  static constexpr const int kNumCameras = 3;
  static constexpr const char* kCameraNames[kNumCameras] = {"left", "center",
                                                            "right"};
  static constexpr const int kLeftCameraIndex = 0;
  static constexpr const int kCenterCameraIndex = 1;
  static constexpr const int kRightCameraIndex = 2;
  static constexpr const int kControllerStateDequeMaxLength = 128;
  static constexpr const int kJointStateDequeMaxLength = 128;
  static constexpr const int kWrenchDequeMaxLength = 128;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::vector<sensor_msgs::msg::Image::UniquePtr> images_;
  std::vector<sensor_msgs::msg::CameraInfo::UniquePtr> camera_infos_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
      image_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr>
      camera_info_subs_;

  std::unique_ptr<std::deque<sensor_msgs::msg::JointState::UniquePtr>>
      joint_state_deque_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;

  std::unique_ptr<
      std::deque<aic_control_interfaces::msg::ControllerState::UniquePtr>>
      controller_state_deque_;
  rclcpp::Subscription<aic_control_interfaces::msg::ControllerState>::SharedPtr
      controller_state_sub_;

  std::unique_ptr<std::deque<geometry_msgs::msg::WrenchStamped::UniquePtr>>
      wrench_deque_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      wrench_sub_;

  rclcpp::Publisher<aic_model_interfaces::msg::Observation>::SharedPtr
      observation_pub_;

  std::unordered_map<std::string, size_t> joint_sort_order_;
};

}  // namespace aic

#endif  // AIC_ADAPTER_HPP_
