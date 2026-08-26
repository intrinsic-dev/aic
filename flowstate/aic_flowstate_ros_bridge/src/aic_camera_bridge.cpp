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

#include "aic_camera_bridge.hpp"

#include <string>
#include <utility>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/perception/proto/v1/capture_result.pb.h"
#include "intrinsic/platform/pubsub/zenoh_util/zenoh_config.h"
#include "intrinsic/util/grpc/channel.h"
#include "intrinsic/util/grpc/connection_params.h"
#include "intrinsic/util/proto/get_text_proto.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/create_service.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/parameter.hpp"

using namespace std::chrono_literals;

namespace flowstate_ros_bridge {

namespace {
constexpr char kLeftCameraOpticalFrame[] = "left_camera/optical";
constexpr char kCenterCameraOpticalFrame[] = "center_camera/optical";
constexpr char kRightCameraOpticalFrame[] = "right_camera/optical";
}  // namespace

///=============================================================================
void AicCameraBridge::declare_ros_parameters(
    ROSNodeInterfaces /*ros_node_interfaces*/) {
  // No parameters currently
}

///=============================================================================
bool AicCameraBridge::initialize(
    ROSNodeInterfaces ros_node_interfaces,
    std::shared_ptr<Executive> /*executive_client*/,
    std::shared_ptr<World> world_client) {
  data_ = std::make_shared<Data>();
  data_->world_client_ = world_client;

  data_->node_interfaces_ = std::move(ros_node_interfaces);

  FindFocalLength();

  data_->pubsub_ = std::make_shared<intrinsic::PubSub>(
      data_->node_interfaces_.get<rclcpp::node_interfaces::NodeBaseInterface>()
          ->get_name());

  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface>
      topics_interface =
          data_->node_interfaces_
              .get<rclcpp::node_interfaces::NodeTopicsInterface>();

  // Reliable QoS subscriptions for images.
  rclcpp::QoS image_pub_qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();

  data_->left_image_pub_ = rclcpp::create_publisher<sensor_msgs::msg::Image>(
      topics_interface, "/left_camera/image", image_pub_qos);
  data_->center_image_pub_ = rclcpp::create_publisher<sensor_msgs::msg::Image>(
      topics_interface, "/center_camera/image", image_pub_qos);
  data_->right_image_pub_ = rclcpp::create_publisher<sensor_msgs::msg::Image>(
      topics_interface, "/right_camera/image", image_pub_qos);

  data_->left_camera_info_pub_ =
      rclcpp::create_publisher<sensor_msgs::msg::CameraInfo>(
          topics_interface, "/left_camera/camera_info", image_pub_qos);
  data_->center_camera_info_pub_ =
      rclcpp::create_publisher<sensor_msgs::msg::CameraInfo>(
          topics_interface, "/center_camera/camera_info", image_pub_qos);
  data_->right_camera_info_pub_ =
      rclcpp::create_publisher<sensor_msgs::msg::CameraInfo>(
          topics_interface, "/right_camera/camera_info", image_pub_qos);

  // Create a Zenoh subscriber on the image pubsub topic
  auto image_sub =
      data_->pubsub_->CreateSubscription<sensor_msgs::msg::pb::jazzy::Image>(
          "cameras/*/image", intrinsic::TopicConfig(),
          [this](const sensor_msgs::msg::pb::jazzy::Image& image_msg) {
            this->ImageCallback(image_msg);
          });
  if (!image_sub.ok()) {
    LOG(ERROR) << "Unable to create Flowstate image subscription: "
               << image_sub.status();
    return false;
  }
  LOG(INFO) << "Subscribed to Flowstate camera image topics";
  data_->image_sub_ =
      std::make_shared<intrinsic::Subscription>(std::move(*image_sub));

  auto left_capture_result_sub =
      data_->pubsub_
          ->CreateSubscription<intrinsic_proto::perception::v1::CaptureResult>(
              "assets/left_camera/capture_result", intrinsic::TopicConfig(),
              [this](const intrinsic_proto::perception::v1::CaptureResult&
                         capture_result_msg) {
                this->CaptureResultCallback(capture_result_msg,
                                            kLeftCameraOpticalFrame,
                                            this->data_->left_image_pub_,
                                            this->data_->left_camera_info_pub_);
              });
  if (!left_capture_result_sub.ok()) {
    LOG(ERROR) << "Unable to create Flowstate capture_result subscription: "
               << left_capture_result_sub.status();
    return false;
  }
  data_->left_capture_result_sub_ = std::make_shared<intrinsic::Subscription>(
      std::move(*left_capture_result_sub));
  auto center_capture_result_sub =
      data_->pubsub_
          ->CreateSubscription<intrinsic_proto::perception::v1::CaptureResult>(
              "assets/center_camera/capture_result", intrinsic::TopicConfig(),
              [this](const intrinsic_proto::perception::v1::CaptureResult&
                         capture_result_msg) {
                this->CaptureResultCallback(
                    capture_result_msg, kCenterCameraOpticalFrame,
                    this->data_->center_image_pub_,
                    this->data_->center_camera_info_pub_);
              });
  if (!center_capture_result_sub.ok()) {
    LOG(ERROR) << "Unable to create Flowstate capture_result subscription: "
               << center_capture_result_sub.status();
    return false;
  }
  data_->center_capture_result_sub_ = std::make_shared<intrinsic::Subscription>(
      std::move(*center_capture_result_sub));

  auto right_capture_result_sub =
      data_->pubsub_
          ->CreateSubscription<intrinsic_proto::perception::v1::CaptureResult>(
              "assets/right_camera/capture_result", intrinsic::TopicConfig(),
              [this](const intrinsic_proto::perception::v1::CaptureResult&
                         capture_result_msg) {
                this->CaptureResultCallback(
                    capture_result_msg, kRightCameraOpticalFrame,
                    this->data_->right_image_pub_,
                    this->data_->right_camera_info_pub_);
              });
  if (!right_capture_result_sub.ok()) {
    LOG(ERROR) << "Unable to create Flowstate capture_result subscription: "
               << right_capture_result_sub.status();
    return false;
  }
  LOG(INFO) << "Subscribed to Flowstate capture_result topics";
  data_->right_capture_result_sub_ = std::make_shared<intrinsic::Subscription>(
      std::move(*right_capture_result_sub));

  LOG(INFO) << "Initialized AicCameraBridge.";

  return true;
}

void AicCameraBridge::CaptureResultCallback(
    const intrinsic_proto::perception::v1::CaptureResult& capture_result,
    const std::string& frame_id,
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> image_pub,
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>
        camera_info_pub) {
  if (capture_result.sensor_images_size() != 1) {
    LOG(ERROR) << " Unexpected number of images: "
               << capture_result.sensor_images_size();
    return;
  }
  if (capture_result.sensor_images(0).buffer().encoding() != 0) {
    LOG(ERROR) << "Expected uncompressed data. Saw encoding type "
               << capture_result.sensor_images(0).buffer().encoding();
    return;
  }
  if (capture_result.sensor_images(0).buffer().pixel_type() != 1) {
    LOG(ERROR) << "Expected pixel type INTENSITY. Saw type "
               << capture_result.sensor_images(0).buffer().pixel_type();
    return;
  }
  if (capture_result.sensor_images(0).buffer().num_channels() != 3) {
    LOG(ERROR) << "Expected a 3-channel image. Saw "
               << capture_result.sensor_images(0).buffer().num_channels();
    return;
  }
  if (capture_result.sensor_images(0).buffer().type() != 1) {
    LOG(ERROR) << "Expected an 8-bit image. Saw data type "
               << capture_result.sensor_images(0).buffer().type();
    return;
  }
  if (!capture_result.sensor_images(0).buffer().has_dimensions()) {
    LOG(ERROR) << "Image buffer did not have dimensions";
    return;
  }
  const int cols = capture_result.sensor_images(0).buffer().dimensions().cols();
  const int rows = capture_result.sensor_images(0).buffer().dimensions().rows();

  sensor_msgs::msg::Image ros_image;
  ros_image.header.frame_id = frame_id;
  ros_image.header.stamp.sec =
      capture_result.sensor_images(0).acquisition_time().seconds();
  ros_image.header.stamp.nanosec =
      capture_result.sensor_images(0).acquisition_time().nanos();
  ros_image.height =
      capture_result.sensor_images(0).buffer().dimensions().rows();
  ros_image.width =
      capture_result.sensor_images(0).buffer().dimensions().cols();
  ros_image.step = ros_image.width * 3;
  ros_image.encoding = "rgb8";
  ros_image.is_bigendian = 0;
  ros_image.data.assign(capture_result.sensor_images(0).buffer().data().begin(),
                        capture_result.sensor_images(0).buffer().data().end());

  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.header = ros_image.header;
  camera_info.height = ros_image.height;
  camera_info.width = ros_image.width;

  camera_info.distortion_model = "plumb_bob";
  camera_info.d.assign(5, 0.0);

  if (capture_result.sensor_images(0).sensor_config().has_camera_params()) {
    const auto& params =
        capture_result.sensor_images(0).sensor_config().camera_params();
    const double fx = params.intrinsic_params().focal_length_x();
    const double fy = params.intrinsic_params().focal_length_y();
    const double cx = params.intrinsic_params().principal_point_x();
    const double cy = params.intrinsic_params().principal_point_y();

    // Store intrinsics in camera_intrinsics_ map
    std::string cam_key;
    if (frame_id.find("left") != std::string::npos) {
      cam_key = "left_camera";
    } else if (frame_id.find("center") != std::string::npos) {
      cam_key = "center_camera";
    } else if (frame_id.find("right") != std::string::npos) {
      cam_key = "right_camera";
    }
    if (!cam_key.empty()) {
      CameraIntrinsics& ci = data_->camera_intrinsics_[cam_key];
      ci.fx = fx;
      ci.fy = fy;
      ci.cx = cx;
      ci.cy = cy;
      ci.valid = true;
    }

    // Populate the intrinsic camera matrix
    camera_info.k[0] = fx;
    camera_info.k[2] = cx;
    camera_info.k[4] = fy;
    camera_info.k[5] = cy;
    camera_info.k[8] = 1.0;

    // Identity rotation matrix, the convention for monocular cameras.
    camera_info.r[0] = 1.0;
    camera_info.r[4] = 1.0;
    camera_info.r[8] = 1.0;

    // Populate the projection matrix following monocular conventions.
    camera_info.p[0] = fx;
    camera_info.p[2] = cx;
    camera_info.p[5] = fy;
    camera_info.p[6] = cy;
    camera_info.p[10] = 1.0;
  }

  image_pub->publish(std::move(ros_image));
  camera_info_pub->publish(std::move(camera_info));
}

///=============================================================================
void AicCameraBridge::ImageCallback(
    const sensor_msgs::msg::pb::jazzy::Image& image) {
  const absl::Time start_time = absl::Now();
  sensor_msgs::msg::Image ros_image;

  // Populate timestamp
  ros_image.header.stamp.sec = image.header().stamp().sec();
  ros_image.header.stamp.nanosec = image.header().stamp().nanosec();
  // frame_id will be populated later, without the incoming numeric ID prefix

  // Populate image metadata
  ros_image.height = image.height();
  ros_image.width = image.width();
  ros_image.encoding = image.encoding();
  ros_image.is_bigendian = image.is_bigendian();
  ros_image.step = image.step();

  // Populate image pixel-block data
  ros_image.data.assign(image.data().begin(), image.data().end());

  const absl::Duration ros_image_creation_duration = absl::Now() - start_time;

  // Route based on frame_id substring
  const std::string& frame_id = image.header().frame_id();
  std::string cam_key;
  if (frame_id.find("left_camera") != std::string::npos) {
    ros_image.header.frame_id = kLeftCameraOpticalFrame;
    cam_key = "left_camera";
  } else if (frame_id.find("center_camera") != std::string::npos) {
    ros_image.header.frame_id = kCenterCameraOpticalFrame;
    cam_key = "center_camera";
  } else if (frame_id.find("right_camera") != std::string::npos) {
    ros_image.header.frame_id = kRightCameraOpticalFrame;
    cam_key = "right_camera";
  } else {
    LOG(WARNING) << "Unknown camera frame_id: " << frame_id;
    return;
  }

  // Every 10 seconds (3 cameras at 20 Hz = 60 images/sec), check to see if
  // any camera intrinsics need to be queried again, in case it failed on startup
  // due to random container startup ordering.
  static int callback_count = 0;
  callback_count++;
  if (callback_count % 600 == 0) {
    bool missing_intrinsics =
        !data_->camera_intrinsics_["left_camera"].valid ||
        !data_->camera_intrinsics_["center_camera"].valid ||
        !data_->camera_intrinsics_["right_camera"].valid;
    if (missing_intrinsics) {
      LOG(INFO) << "Camera intrinsics still incomplete. Retrying search.";
      FindFocalLength();
    }
    LOG(INFO) << absl::StrFormat(
        "camera image translation took %.6f seconds",
        absl::ToDoubleSeconds(ros_image_creation_duration));
  }

  CameraIntrinsics intrinsics;
  auto it = data_->camera_intrinsics_.find(cam_key);
  if (it != data_->camera_intrinsics_.end() && it->second.valid) {
    intrinsics = it->second;
  } else {
    // Fallback if not yet received
    intrinsics.cx = ros_image.width / 2.0;
    intrinsics.cy = ros_image.height / 2.0;
  }

  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.header = ros_image.header;
  camera_info.height = ros_image.height;
  camera_info.width = ros_image.width;

  // Images from the simulation are undistorted unless distortion is specified
  camera_info.distortion_model = "plumb_bob";
  if (!intrinsics.d.empty()) {
    camera_info.d = intrinsics.d;
  } else {
    camera_info.d.assign(5, 0.0);
  }

  // Populate the intrinsic camera matrix
  camera_info.k[0] = intrinsics.fx;
  camera_info.k[2] = intrinsics.cx;
  camera_info.k[4] = intrinsics.fy;
  camera_info.k[5] = intrinsics.cy;
  camera_info.k[8] = 1.0;

  // Identity rotation matrix, the convention for monocular cameras.
  camera_info.r[0] = 1.0;
  camera_info.r[4] = 1.0;
  camera_info.r[8] = 1.0;

  // Populate the projection matrix following monocular conventions.
  camera_info.p[0] = intrinsics.fx;
  camera_info.p[2] = intrinsics.cx;
  camera_info.p[5] = intrinsics.fy;
  camera_info.p[6] = intrinsics.cy;
  camera_info.p[10] = 1.0;

  if (cam_key == "left_camera") {
    data_->left_image_pub_->publish(std::move(ros_image));
    data_->left_camera_info_pub_->publish(std::move(camera_info));
  } else if (cam_key == "center_camera") {
    data_->center_image_pub_->publish(std::move(ros_image));
    data_->center_camera_info_pub_->publish(std::move(camera_info));
  } else if (cam_key == "right_camera") {
    data_->right_image_pub_->publish(std::move(ros_image));
    data_->right_camera_info_pub_->publish(std::move(camera_info));
  }
}

///=============================================================================
AicCameraBridge::Data::Data() {}

AicCameraBridge::Data::~Data() {
  pubsub_.reset();
  image_sub_.reset();
  left_capture_result_sub_.reset();
  center_capture_result_sub_.reset();
  right_capture_result_sub_.reset();
  left_image_pub_.reset();
  center_image_pub_.reset();
  right_image_pub_.reset();
}

void AicCameraBridge::FindFocalLength() {
  if (!data_->world_client_) return;
  auto objects_or = data_->world_client_->GetObjects();
  if (!objects_or.ok()) {
    LOG(WARNING) << "Failed to query objects from World service: "
                 << objects_or.status();
    return;
  }
  for (const auto& object : *objects_or) {
    const auto& proto = object.Proto();
    for (const auto& [name, entity] : proto.entities()) {
      if (entity.has_sensor_component() &&
          entity.sensor_component().has_camera()) {
        const auto& camera = entity.sensor_component().camera();
        if (camera.has_properties() && camera.properties().has_intrinsics()) {
          const auto& intr = camera.properties().intrinsics();
          CameraIntrinsics ci;
          ci.fx = intr.fx();
          ci.fy = intr.fy();
          ci.cx = intr.cx();
          ci.cy = intr.cy();
          if (camera.properties().has_distortion()) {
            const auto& dist = camera.properties().distortion();
            ci.d = {dist.k1(), dist.k2(), dist.p1(), dist.p2(), dist.k3()};
          }
          ci.valid = true;

          std::string entity_name = entity.name();
          if (entity_name.empty()) entity_name = entity.local_name();
          if (entity_name.empty()) entity_name = entity.alias();
          if (entity_name.empty()) entity_name = entity.sensor_component().topic();

          std::string cam_key;
          if (entity_name.find("left") != std::string::npos) {
            cam_key = "left_camera";
          } else if (entity_name.find("center") != std::string::npos) {
            cam_key = "center_camera";
          } else if (entity_name.find("right") != std::string::npos) {
            cam_key = "right_camera";
          } else if (!entity_name.empty()) {
            cam_key = entity_name;
          }

          if (!cam_key.empty()) {
            data_->camera_intrinsics_[cam_key] = ci;
            LOG(INFO) << "Found camera [" << cam_key << "] (entity: '"
                      << entity_name << "') intrinsics via gRPC: fx=" << ci.fx
                      << ", fy=" << ci.fy << ", cx=" << ci.cx
                      << ", cy=" << ci.cy;
          }
        }
      }
    }
  }
}

}  // namespace flowstate_ros_bridge

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(flowstate_ros_bridge::AicCameraBridge,
                       flowstate_ros_bridge::BridgeInterface)
