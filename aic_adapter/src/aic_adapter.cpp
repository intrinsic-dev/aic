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

#include "aic_adapter/aic_adapter.hpp"

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

template <typename T>
bool ReorderJointArray(const std::vector<T>& original,
                       const std::vector<size_t>& ordering,
                       std::vector<T>& reordered) {
  if (original.empty()) {
    reordered.clear();
    return true;
  }

  const size_t n_joints = original.size();
  if (n_joints != ordering.size()) {
    return false;
  }

  reordered.resize(n_joints);

  return true;
}

AicAdapterNode::AicAdapterNode() : Node("aic_adapter_node") {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  images_.resize(kNumCameras);
  camera_infos_.resize(kNumCameras);

  observation_pub_ =
      this->create_publisher<aic_model_interfaces::msg::Observation>(
          "observations", 10);

  wrench_deque_ = std::make_unique<
      std::deque<geometry_msgs::msg::WrenchStamped::UniquePtr>>();
  wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/fts_broadcaster/wrench", 5,
      [this](geometry_msgs::msg::WrenchStamped::UniquePtr msg) -> void {
        this->wrench_deque_->push_front(std::move(msg));
        while (this->wrench_deque_->size() > kWrenchDequeMaxLength) {
          this->wrench_deque_->pop_back();
        }
      });

  joint_sort_order_["shoulder_pan_joint"] = 0;
  joint_sort_order_["shoulder_lift_joint"] = 1;
  joint_sort_order_["elbow_joint"] = 2;
  joint_sort_order_["wrist_1_joint"] = 3;
  joint_sort_order_["wrist_2_joint"] = 4;
  joint_sort_order_["wrist_3_joint"] = 5;
  joint_sort_order_["gripper/left_finger_joint"] = 6;
  joint_state_deque_ =
      std::make_unique<std::deque<sensor_msgs::msg::JointState::UniquePtr>>();
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 5,
      [this](sensor_msgs::msg::JointState::UniquePtr msg) -> void {
        this->joint_state_deque_->push_front(std::move(msg));
        while (this->joint_state_deque_->size() > kJointStateDequeMaxLength) {
          this->joint_state_deque_->pop_back();
        }
      });

  controller_state_deque_ = std::make_unique<
      std::deque<aic_control_interfaces::msg::ControllerState::UniquePtr>>();
  controller_state_sub_ =
      this->create_subscription<aic_control_interfaces::msg::ControllerState>(
          "/aic_controller/controller_state", 5,
          [this](aic_control_interfaces::msg::ControllerState::UniquePtr msg)
              -> void {
            this->controller_state_deque_->push_front(std::move(msg));
            while (this->controller_state_deque_->size() >
                   kControllerStateDequeMaxLength) {
              this->controller_state_deque_->pop_back();
            }
          });

  for (size_t camera_idx = 0; camera_idx < kNumCameras; camera_idx++) {
    camera_info_subs_.push_back(
        this->create_subscription<sensor_msgs::msg::CameraInfo>(
            std::format("/{}_camera/camera_info", kCameraNames[camera_idx]), 5,
            [this, camera_idx](sensor_msgs::msg::CameraInfo::UniquePtr msg)
                -> void { this->camera_infos_[camera_idx] = std::move(msg); }));
    image_subs_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
        std::format("/{}_camera/image", kCameraNames[camera_idx]), 5,
        [this, camera_idx](sensor_msgs::msg::Image::UniquePtr msg) -> void {
          this->image_callback(camera_idx, std::move(msg));
        }));
  }
  RCLCPP_INFO(this->get_logger(), "Adapter node initialization complete.");
}

void AicAdapterNode::image_callback(size_t camera_idx,
                                    sensor_msgs::msg::Image::UniquePtr msg) {
  if (camera_idx > images_.size()) {
    RCLCPP_ERROR(this->get_logger(), "unexpected camera idx: %zu", camera_idx);
    return;
  }
  images_[camera_idx] = std::move(msg);

  // See if we have a recent image collection. If so, re-publish them and
  // remove them from our buffer.
  for (size_t i = 0; i < kNumCameras; i++) {
    if (!images_[i] || !camera_infos_[i]) {
      return;
    }
  }

  const rclcpp::Time t_image_0(images_[0]->header.stamp);
  for (size_t i = 1; i < kNumCameras; i++) {
    const rclcpp::Duration cam_time_diff =
        t_image_0 - rclcpp::Time(images_[i]->header.stamp);
    if (abs(cam_time_diff.seconds()) > 0.001) {
      return;
    }
  }

  // If we get here, all of the camera image timestamps are aligned
  aic_model_interfaces::msg::Observation::UniquePtr observation_msg =
      std::make_unique<aic_model_interfaces::msg::Observation>();

  observation_msg->left_image = std::move(*images_[kLeftCameraIndex]);
  observation_msg->center_image = std::move(*images_[kCenterCameraIndex]);
  observation_msg->right_image = std::move(*images_[kRightCameraIndex]);

  // Make a copy of the CameraInfos, in case we need the original again
  // during the next image cycle.
  observation_msg->left_camera_info = *camera_infos_[kLeftCameraIndex];
  observation_msg->center_camera_info = *camera_infos_[kCenterCameraIndex];
  observation_msg->right_camera_info = *camera_infos_[kRightCameraIndex];

  // Because we know the CameraInfo structs are not changing (these are
  // fixed-focus cameras), update the timestamp to match the images.
  // (This is to handle any randomness in the arrival order of the image
  // and its associated CameraInfo.)
  observation_msg->left_camera_info.header.stamp =
      observation_msg->left_image.header.stamp;
  observation_msg->center_camera_info.header.stamp =
      observation_msg->center_image.header.stamp;
  observation_msg->right_camera_info.header.stamp =
      observation_msg->right_image.header.stamp;

  // Look for the joint state message that is closest to the timestamp
  // of the images.
  size_t joint_state_msg_idx = 0;
  for (joint_state_msg_idx = 0;
       joint_state_msg_idx < joint_state_deque_->size();
       joint_state_msg_idx++) {
    if (!(*joint_state_deque_)[joint_state_msg_idx]) {
      continue;
    }
    const rclcpp::Time t_joint_state_msg(
        (*joint_state_deque_)[joint_state_msg_idx]->header.stamp);
    if (t_joint_state_msg <= t_image_0) {
      ReorderJointState(*(*joint_state_deque_)[joint_state_msg_idx],
                        observation_msg->joint_states);
      break;
    }
  }

  // Look for the controller state message that is closest to the timestamp
  // of the images.
  size_t controller_state_msg_idx = 0;
  for (controller_state_msg_idx = 0;
       controller_state_msg_idx < controller_state_deque_->size();
       controller_state_msg_idx++) {
    if (!(*controller_state_deque_)[controller_state_msg_idx]) {
      continue;
    }
    const rclcpp::Time t_controller_state_msg(
        (*controller_state_deque_)[controller_state_msg_idx]->header.stamp);
    if (t_controller_state_msg <= t_image_0) {
      observation_msg->controller_state =
          *(*controller_state_deque_)[controller_state_msg_idx];
      break;
    }
  }

  // Look for the wrench message that is closest to the timestamp
  // of the images.
  size_t wrench_msg_idx = 0;
  for (wrench_msg_idx = 0; wrench_msg_idx < wrench_deque_->size();
       wrench_msg_idx++) {
    if (!(*wrench_deque_)[wrench_msg_idx]) {
      continue;
    }
    const rclcpp::Time t_wrench_msg(
        (*wrench_deque_)[wrench_msg_idx]->header.stamp);
    if (t_wrench_msg <= t_image_0) {
      observation_msg->wrist_wrench = *(*wrench_deque_)[wrench_msg_idx];
      break;
    }
  }

  this->observation_pub_->publish(std::move(observation_msg));
}

void AicAdapterNode::ReorderJointState(
    const sensor_msgs::msg::JointState& original,
    sensor_msgs::msg::JointState& reordered) {
  reordered.header = original.header;
  const size_t n_joints = original.name.size();
  if (n_joints != joint_sort_order_.size()) {
    RCLCPP_ERROR(get_logger(), "Expected %zu joints. Received %zu",
                 joint_sort_order_.size(), n_joints);
    return;
  }

  if (original.position.size() != n_joints) {
    RCLCPP_ERROR(get_logger(), "Expected %zu joint positions. Received %zu",
                 original.position.size(), n_joints);
    return;
  }

  if (original.velocity.size() != n_joints) {
    RCLCPP_ERROR(get_logger(), "Expected %zu joint velocities. Received %zu",
                 original.velocity.size(), n_joints);
    return;
  }

  if (original.effort.size() != n_joints) {
    RCLCPP_ERROR(get_logger(), "Expected %zu joint efforts. Received %zu",
                 original.effort.size(), n_joints);
    return;
  }

  reordered.name.resize(n_joints);
  reordered.position.resize(n_joints);
  reordered.velocity.resize(n_joints);
  reordered.effort.resize(n_joints);

  for (size_t original_joint_idx = 0; original_joint_idx < n_joints;
       original_joint_idx++) {
    if (!joint_sort_order_.contains(original.name[original_joint_idx])) {
      RCLCPP_ERROR(get_logger(), "Ignoring unexpected joint name: %s",
                   original.name[original_joint_idx].c_str());
      continue;
    }
    const size_t reordered_idx =
        joint_sort_order_.at(original.name[original_joint_idx]);
    reordered.name[reordered_idx] = original.name[original_joint_idx];
    reordered.position[reordered_idx] = original.position[original_joint_idx];
    reordered.velocity[reordered_idx] = original.velocity[original_joint_idx];
    reordered.effort[reordered_idx] = original.effort[original_joint_idx];
  }

  // Rename the last joint "gripper", and change it to the distance between
  // the fingers, rather than the prismatic joint motion, just by dividing
  // the value by 2.
  reordered.name[n_joints - 1] = "gripper";
  reordered.position[n_joints - 1] /= 2.0;
  reordered.velocity[n_joints - 1] /= 2.0;
  reordered.effort[n_joints - 1] /= 2.0;
}

}  // namespace aic

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aic::AicAdapterNode>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
