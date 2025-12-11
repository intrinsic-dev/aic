#include <cstdlib>
#include <cstring>
#include <deque>
#include <format>
#include <memory>

#include "aic_model_interfaces/msg/observation.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class AicAdapterNode : public rclcpp::Node {
 public:
  AicAdapterNode() : Node("aic_adapter_node") {
    RCLCPP_INFO(this->get_logger(), "Hello, world!");
    images_.resize(kNumCameras);
    camera_infos_.resize(kNumCameras);

    observation_pub_ =
        this->create_publisher<aic_model_interfaces::msg::Observation>(
            "observations", 10);

    wrench_deque_ = std::make_unique<
        std::deque<geometry_msgs::msg::WrenchStamped::UniquePtr>>();
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/axia80_m20/wrench", 5,
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

    for (size_t camera_idx = 0; camera_idx < kNumCameras; camera_idx++) {
      camera_info_subs_.push_back(
          this->create_subscription<sensor_msgs::msg::CameraInfo>(
              std::format("/wrist_camera_{}/camera_info", camera_idx + 1), 5,
              [this, camera_idx](
                  sensor_msgs::msg::CameraInfo::UniquePtr msg) -> void {
                this->camera_infos_[camera_idx] = std::move(msg);
              }));
      image_subs_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
          std::format("/wrist_camera_{}/image", camera_idx + 1), 5,
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
      if (i >= observation_msg->images.size()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Tried to publish an unknown wrist camera: %zu", i);
        continue;
      }
      observation_msg->images[i] = std::move(*images_[i]);
      if (camera_infos_[i]) {
        // Make a copy of this CameraInfo, in case we need the original again
        // during the next image cycle.
        observation_msg->camera_infos[i] = *camera_infos_[i];
        // Because we know the CameraInfo structs are not changing (these are
        // fixed-focus cameras), update the timestamp to match the images.
        // (This is to handle any randomness in the arrival order of the image
        // and its associated CameraInfo.)
        observation_msg->camera_infos[i].header.stamp =
            observation_msg->images[i].header.stamp;
      }
    }

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
        SortJointStateMessage(*(*joint_state_deque_)[joint_state_msg_idx],
                              observation_msg->joint_states);
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
        observation_msg->wrench = *(*wrench_deque_)[wrench_msg_idx];
        break;
      }
    }

    this->observation_pub_->publish(std::move(observation_msg));
  }

  void SortJointStateMessage(const sensor_msgs::msg::JointState& unsorted,
                             sensor_msgs::msg::JointState& sorted) {
    sorted.header = unsorted.header;
    const size_t n_joints = unsorted.name.size();
    if (n_joints != joint_sort_order_.size()) {
      RCLCPP_ERROR(get_logger(), "Expected %zu joints. Received %zu",
                   joint_sort_order_.size(), n_joints);
      return;
    }

    if (unsorted.position.size() != n_joints) {
      RCLCPP_ERROR(get_logger(), "Expected %zu joint positions. Received %zu",
                   unsorted.position.size(), n_joints);
      return;
    }

    if (unsorted.velocity.size() != n_joints) {
      RCLCPP_ERROR(get_logger(), "Expected %zu joint velocities. Received %zu",
                   unsorted.velocity.size(), n_joints);
      return;
    }

    if (unsorted.effort.size() != n_joints) {
      RCLCPP_ERROR(get_logger(), "Expected %zu joint efforts. Received %zu",
                   unsorted.effort.size(), n_joints);
      return;
    }

    sorted.name.resize(n_joints);
    sorted.position.resize(n_joints);
    sorted.velocity.resize(n_joints);
    sorted.effort.resize(n_joints);

    for (size_t unsorted_joint_idx = 0; unsorted_joint_idx < n_joints;
         unsorted_joint_idx++) {
      if (!joint_sort_order_.contains(unsorted.name[unsorted_joint_idx])) {
        RCLCPP_ERROR(get_logger(), "Ignoring unexpected joint name: %s",
                     unsorted.name[unsorted_joint_idx].c_str());
        continue;
      }
      const size_t sorted_idx =
          joint_sort_order_.at(unsorted.name[unsorted_joint_idx]);
      sorted.name[sorted_idx] = unsorted.name[unsorted_joint_idx];
      sorted.position[sorted_idx] = unsorted.position[unsorted_joint_idx];
      sorted.velocity[sorted_idx] = unsorted.velocity[unsorted_joint_idx];
      sorted.effort[sorted_idx] = unsorted.effort[unsorted_joint_idx];
    }

    // Rename the last joint "gripper", and change it to the distance between
    // the fingers, rather than the prismatic joint motion, just by dividing
    // the value by 2. Taken from this definition, the "distance between the
    // gripper fingers" has a velocity twice that of each individual finger.
    sorted.name[n_joints - 1] = "gripper";
    sorted.position[n_joints - 1] /= 2.0;
    sorted.velocity[n_joints - 1] *= 2.0;
  }

  static const int kNumCameras = 3;
  static const int kJointStateDequeMaxLength = 128;
  static const int kWrenchDequeMaxLength = 128;
  rclcpp::TimerBase::SharedPtr timer_;

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

  std::unique_ptr<std::deque<geometry_msgs::msg::WrenchStamped::UniquePtr>>
      wrench_deque_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      wrench_sub_;

  rclcpp::Publisher<aic_model_interfaces::msg::Observation>::SharedPtr
      observation_pub_;

  std::unordered_map<std::string, size_t> joint_sort_order_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AicAdapterNode>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
