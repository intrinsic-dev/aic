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

#ifndef BRIDGES_AIC_CAMERA_BRIDGE_HPP_
#define BRIDGES_AIC_CAMERA_BRIDGE_HPP_

#include <memory>
#include <mutex>

#include "flowstate_ros_bridge/bridge_interface.hpp"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "third_party/ros2/ros_interfaces/jazzy/sensor_msgs/msg/image.pb.h"

namespace flowstate_ros_bridge {

class AicCameraBridge : public BridgeInterface {
 public:
  /// Documentation inherited.
  void declare_ros_parameters(ROSNodeInterfaces ros_node_interfaces) final;

  /// Documentation inherited.
  bool initialize(ROSNodeInterfaces ros_node_interfaces,
                  std::shared_ptr<Executive> executive_client,
                  std::shared_ptr<World> world_client) final;

 private:
  void FindFocalLength();
  void ImageCallback(const sensor_msgs::msg::pb::jazzy::Image& image);

  struct Data : public std::enable_shared_from_this<Data> {
    ROSNodeInterfaces node_interfaces_;

    std::shared_ptr<intrinsic::PubSub> pubsub_;
    std::shared_ptr<World> world_client_;
    double focal_length_x_ = 0.0;
    double focal_length_y_ = 0.0;
    std::shared_ptr<intrinsic::Subscription> image_sub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> left_image_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>
        center_image_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>
        right_image_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>
        left_camera_info_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>
        center_camera_info_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>
        right_camera_info_pub_;

    Data();
    ~Data();
  };
  std::shared_ptr<Data> data_;
};

}  // namespace flowstate_ros_bridge.

#endif  // BRIDGES_AIC_CAMERA_BRIDGE_HPP_
