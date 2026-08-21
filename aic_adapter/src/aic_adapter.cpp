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

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <format>
#include <memory>
#include <string>

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
  include_gripper_in_joint_state_ =
      this->declare_parameter("include_gripper_in_joint_state", true);
  image_time_tolerance_ =
      this->declare_parameter("image_time_tolerance", 0.001);
  align_camera_timestamps_ =
      this->declare_parameter("align_camera_timestamps", false);

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
        RCLCPP_INFO_ONCE(this->get_logger(),
                         "Received first wrench message from /fts_broadcaster/wrench");
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
  if (include_gripper_in_joint_state_) {
    joint_sort_order_["gripper/left_finger_joint"] = 6;
  }
  joint_state_deque_ =
      std::make_unique<std::deque<sensor_msgs::msg::JointState::UniquePtr>>();
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 5,
      [this](sensor_msgs::msg::JointState::UniquePtr msg) -> void {
        RCLCPP_INFO_ONCE(this->get_logger(),
                         "Received first joint_state message from /joint_states");
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
            RCLCPP_INFO_ONCE(
                this->get_logger(),
                "Received first controller_state message from /aic_controller/controller_state");
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
                -> void {
              RCLCPP_INFO_ONCE(
                  this->get_logger(),
                  "Received first camera_info message for %s_camera",
                  kCameraNames[camera_idx]);
              this->camera_infos_[camera_idx] = std::move(msg);
            }));
    image_subs_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
        std::format("/{}_camera/image", kCameraNames[camera_idx]), 5,
        [this, camera_idx](sensor_msgs::msg::Image::UniquePtr msg) -> void {
          RCLCPP_INFO_ONCE(
              this->get_logger(),
              "Received first image message for %s_camera (stamp: %.6f)",
              kCameraNames[camera_idx],
              rclcpp::Time(msg->header.stamp).seconds());
          this->image_callback(camera_idx, std::move(msg));
        }));
  }

  RCLCPP_INFO(this->get_logger(),
              "Adapter node initialization complete. Params: "
              "image_time_tolerance=%.6f s, align_camera_timestamps=%s, "
              "include_gripper_in_joint_state=%s. Publishing topic: '%s'",
              image_time_tolerance_,
              align_camera_timestamps_ ? "true" : "false",
              include_gripper_in_joint_state_ ? "true" : "false",
              observation_pub_->get_topic_name());
}

void AicAdapterNode::image_callback(size_t camera_idx,
                                    sensor_msgs::msg::Image::UniquePtr msg) {
  if (camera_idx >= images_.size()) {
    RCLCPP_ERROR(this->get_logger(), "unexpected camera idx: %zu (max %zu)",
                 camera_idx, images_.size());
    return;
  }
  images_[camera_idx] = std::move(msg);

  // See if we have a recent image collection. If so, re-publish them and
  // remove them from our buffer.
  for (size_t i = 0; i < kNumCameras; i++) {
    if (!images_[i] || !camera_infos_[i]) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[image_callback] Waiting for all camera data. Status: "
          "left(img=%s, info=%s), center(img=%s, info=%s), right(img=%s, info=%s)",
          images_[kLeftCameraIndex] ? "OK" : "MISSING",
          camera_infos_[kLeftCameraIndex] ? "OK" : "MISSING",
          images_[kCenterCameraIndex] ? "OK" : "MISSING",
          camera_infos_[kCenterCameraIndex] ? "OK" : "MISSING",
          images_[kRightCameraIndex] ? "OK" : "MISSING",
          camera_infos_[kRightCameraIndex] ? "OK" : "MISSING");
      return;
    }
  }

  const rclcpp::Time t_image_0(images_[0]->header.stamp);
  for (size_t i = 1; i < kNumCameras; i++) {
    const rclcpp::Duration cam_time_diff =
        t_image_0 - rclcpp::Time(images_[i]->header.stamp);
    const double diff_sec = std::abs(cam_time_diff.seconds());
    if (diff_sec > image_time_tolerance_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[image_callback] Camera timestamp mismatch! Delta between %s and %s is %.6f s "
          "(tolerance: %.6f s). Stamps: %s=%.6f, %s=%.6f",
          kCameraNames[0], kCameraNames[i], diff_sec, image_time_tolerance_,
          kCameraNames[0], t_image_0.seconds(),
          kCameraNames[i], rclcpp::Time(images_[i]->header.stamp).seconds());
      return;
    }
  }

  // If we get here, all of the camera image timestamps are aligned
  aic_model_interfaces::msg::Observation::UniquePtr observation_msg =
      std::make_unique<aic_model_interfaces::msg::Observation>();

  observation_msg->left_image = std::move(*images_[kLeftCameraIndex]);
  observation_msg->center_image = std::move(*images_[kCenterCameraIndex]);
  observation_msg->right_image = std::move(*images_[kRightCameraIndex]);

  // Optionally force all camera timestamps to be exactly aligned
  if (align_camera_timestamps_) {
    observation_msg->left_image.header.stamp =
        observation_msg->center_image.header.stamp;
    observation_msg->right_image.header.stamp =
        observation_msg->center_image.header.stamp;
  }

  images_[kLeftCameraIndex].reset();
  images_[kCenterCameraIndex].reset();
  images_[kRightCameraIndex].reset();

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
  bool joint_state_found = false;
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
      joint_state_found = true;
      break;
    }
  }
  if (!joint_state_found) {
    if (joint_state_deque_->empty()) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[image_callback] Joint state deque is empty. Observation joint_states will be empty.");
    } else {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[image_callback] No joint_state found with stamp <= image stamp "
          "(image_t=%.6f, newest_joint_t=%.6f, oldest_joint_t=%.6f).",
          t_image_0.seconds(),
          rclcpp::Time((*joint_state_deque_)[0]->header.stamp).seconds(),
          rclcpp::Time(joint_state_deque_->back()->header.stamp).seconds());
    }
  }

  // Look for the controller state message that is closest to the timestamp
  // of the images.
  size_t controller_state_msg_idx = 0;
  bool controller_state_found = false;
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
      controller_state_found = true;
      break;
    }
  }
  if (!controller_state_found) {
    if (controller_state_deque_->empty()) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[image_callback] Controller state deque is empty.");
    } else {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[image_callback] No controller_state found with stamp <= image stamp "
          "(image_t=%.6f, newest_ctrl_t=%.6f, oldest_ctrl_t=%.6f).",
          t_image_0.seconds(),
          rclcpp::Time((*controller_state_deque_)[0]->header.stamp).seconds(),
          rclcpp::Time(controller_state_deque_->back()->header.stamp).seconds());
    }
  }

  // Look for the wrench message that is closest to the timestamp
  // of the images.
  size_t wrench_msg_idx = 0;
  bool wrench_found = false;
  for (wrench_msg_idx = 0; wrench_msg_idx < wrench_deque_->size();
       wrench_msg_idx++) {
    if (!(*wrench_deque_)[wrench_msg_idx]) {
      continue;
    }
    const rclcpp::Time t_wrench_msg(
        (*wrench_deque_)[wrench_msg_idx]->header.stamp);
    if (t_wrench_msg <= t_image_0) {
      observation_msg->wrist_wrench = *(*wrench_deque_)[wrench_msg_idx];
      wrench_found = true;
      break;
    }
  }
  if (!wrench_found) {
    if (wrench_deque_->empty()) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[image_callback] Wrench deque is empty.");
    } else {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[image_callback] No wrench found with stamp <= image stamp "
          "(image_t=%.6f, newest_wrench_t=%.6f, oldest_wrench_t=%.6f).",
          t_image_0.seconds(),
          rclcpp::Time((*wrench_deque_)[0]->header.stamp).seconds(),
          rclcpp::Time(wrench_deque_->back()->header.stamp).seconds());
    }
  }

  this->observation_pub_->publish(std::move(observation_msg));
  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Successfully published /observations message (image stamp: %.6f).",
      t_image_0.seconds());
}

void AicAdapterNode::ReorderJointState(
    const sensor_msgs::msg::JointState& original,
    sensor_msgs::msg::JointState& reordered) {
  reordered.header = original.header;
  const size_t n_joints = original.name.size();
  if (n_joints != joint_sort_order_.size()) {
    std::string received_names;
    for (const auto& name : original.name) {
      received_names += name + " ";
    }
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[ReorderJointState] Expected %zu joints, received %zu. Joints in message: [%s]",
        joint_sort_order_.size(), n_joints, received_names.c_str());
    return;
  }

  if (original.position.size() != n_joints) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[ReorderJointState] Expected %zu joint positions. Received %zu",
        original.position.size(), n_joints);
    return;
  }

  if (original.velocity.size() != n_joints) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[ReorderJointState] Expected %zu joint velocities. Received %zu",
        original.velocity.size(), n_joints);
    return;
  }

  if (original.effort.size() != n_joints) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[ReorderJointState] Expected %zu joint efforts. Received %zu",
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
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[ReorderJointState] Ignoring unexpected joint name: %s",
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

  if (include_gripper_in_joint_state_) {
    // Rename the last joint "gripper", and change it to the distance between
    // the fingers, rather than the prismatic joint motion, just by dividing
    // the value by 2.
    reordered.name[n_joints - 1] = "gripper";
    reordered.position[n_joints - 1] /= 2.0;
    reordered.velocity[n_joints - 1] /= 2.0;
    reordered.effort[n_joints - 1] /= 2.0;
  }
}

}  // namespace aic

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aic::AicAdapterNode>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
