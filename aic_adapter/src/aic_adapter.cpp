#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>

#include "aic_model_interfaces/msg/observation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class AicAdapterNode : public rclcpp::Node {
 public:
  AicAdapterNode() : Node("aic_adapter_node") {
    RCLCPP_INFO(this->get_logger(), "Hello, world!");
    images_.resize(kNumCameras);
    observation_pub_ =
        this->create_publisher<aic_model_interfaces::msg::Observation>(
            "observations", 10);
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
    for (size_t camera_idx = 0; camera_idx < kNumCameras; camera_idx++) {
      char topic_name_buf[64] = {0};
      snprintf(topic_name_buf, sizeof(topic_name_buf),
               "/wrist_camera_%zu/image", camera_idx + 1);
      image_subs_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
          topic_name_buf, 5,
          [this, camera_idx](sensor_msgs::msg::Image::UniquePtr msg) -> void {
            this->image_callback(camera_idx, std::move(msg));
          }));
    }
  }
  virtual ~AicAdapterNode() {}
 
 private:
  void image_callback(size_t camera_idx,
                      sensor_msgs::msg::Image::UniquePtr msg) {
    if (camera_idx > images_.size()) {
      RCLCPP_ERROR(this->get_logger(), "unexpected camera idx: %zu",
                   camera_idx);
      return;
    }
    images_[camera_idx] = std::move(msg);

    // See if we have a recent image collection. If so, re-publish them and
    // remove them from our buffer.
    for (size_t i = 0; i < kNumCameras; i++) {
      if (!images_[i]) {
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

    for (size_t i = 0; i < kNumCameras; i++) {
      if (i >= observation_msg->wrist_cameras.size()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Tried to publish an unknown wrist camera: %zu", i);
        continue;
      }
      observation_msg->wrist_cameras[i] =
          std::move(*images_[i]);
    }

    // Avoid a copy by moving the last element into our outbound message
    // and deleting the empty message that results from the move.
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
        observation_msg->joint_states =
            std::move(*(*joint_state_deque_)[joint_state_msg_idx]);
        break;
      }
    }

    this->observation_pub_->publish(std::move(observation_msg));
  }

  static const int kNumCameras = 3;
  static const int kJointStateDequeMaxLength = 128;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
      image_subs_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  std::vector<sensor_msgs::msg::Image::UniquePtr> images_;
  std::unique_ptr<std::deque<sensor_msgs::msg::JointState::UniquePtr>>
      joint_state_deque_;
  rclcpp::Publisher<aic_model_interfaces::msg::Observation>::SharedPtr
      observation_pub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AicAdapterNode>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
