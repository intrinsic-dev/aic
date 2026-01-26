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

#ifndef AIC_SCORING__SCORING_TIER2_HH_
#define AIC_SCORING__SCORING_TIER2_HH_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <gz/math/Pose3.hh>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace aic_scoring
{
  /// \brief Connection POD.
  struct Connection
  {
    /// \brief Plug name.
    public: std::string plugName;

    /// \brief Port name.
    public: std::string portName;

    /// \brief Plug/port type.
    public: std::string type;

    /// \brief Distance.
    public: double distance = -1;
  };

  /// \brief Topic info POD.
  struct TopicInfo
  {
    /// \brief Topic name.
    std::string name;

    /// \brief Topic type (e.g., sensor_msgs/msg/JointState).
    std::string type;
  };

  // The Tier2 scoring interface.
  class ScoringTier2
  {
    /// \brief Class constructor.
    /// \param[in] _node Pointer to the ROS node.
    public: ScoringTier2(rclcpp::Node *_node);

    /// \brief Populate the scoring input params from a YAML file.
    /// \param[in] _config YAML configuration for the node
    public: bool Initialize(YAML::Node _config);

    /// \brief Reset connections.
    /// \param[in] _connections New connections.
    public: void ResetConnections(const std::vector<Connection> &_connections);

    /// \brief Start recording all scoring topics.
    /// \return True if the bag was opened correctly and it's ready to record.
    /// \param[in] _filename The path to the bag.
    public: bool StartRecording(const std::string &_filename);

    /// \brief Stop recording all scoring topics.
    /// \return True if the bag was closed correctly.
    public: bool StopRecording();

    /// \brief Populate the scoring input params from a YAML file.
    /// \param[in] _config YAML configuration for the node
    private: bool ParseStats(YAML::Node _config);

    /// \brief Update jerk computation with a new pose sample.
    /// \param[in] _pose The new timestamped pose.
    /// \return True if successful, false if timestamp was not increasing.
    public: bool UpdateJerk(const geometry_msgs::msg::PoseStamped &_pose);

    /// \brief Get the current linear jerk.
    /// \return The linear jerk vector (x, y, z) in m/s^3.
    public: geometry_msgs::msg::Vector3 GetLinearJerk() const;

    /// \brief Get the current angular jerk.
    /// \return The angular jerk vector (roll, pitch, yaw) in rad/s^3.
    public: geometry_msgs::msg::Vector3 GetAngularJerk() const;

    /// \brief Get the time-weighted average linear jerk.
    /// \return The average linear jerk vector (x, y, z) in m/s^3.
    public: geometry_msgs::msg::Vector3 GetAvgLinearJerk() const;

    /// \brief Get the time-weighted average angular jerk.
    /// \return The average angular jerk vector (roll, pitch, yaw) in rad/s^3.
    public: geometry_msgs::msg::Vector3 GetAvgAngularJerk() const;

    /// \brief Reset the jerk computation state.
    public: void ResetJerk();

    /// \brief Update plug-port connection distance computation.
    /// \param[in] _plug The plug point (timestamped).
    /// \param[in] _port The port point (timestamped).
    /// \return True if successful, false if timestamp was not increasing.
    public: bool UpdatePlugPortDistance(
        const geometry_msgs::msg::PointStamped &_plug,
        const geometry_msgs::msg::PointStamped &_port);

    /// \brief Get the current plug-port connection distance.
    /// \return The Euclidean distance in meters.
    public: double GetPlugPortDistance() const;

    /// \brief Get the time-weighted average plug-port connection distance.
    /// \return The average distance in meters.
    public: double GetAvgPlugPortDistance() const;

    /// \brief Reset the plug-port distance computation state.
    public: void ResetPlugPortDistance();

    /// \brief Pointer to a node.
    private: rclcpp::Node *node;

    /// \brief Topics to subscribe to.
    private: std::vector<TopicInfo> topics;

    /// \brief Connections.
    private: std::vector<Connection> connections;

    /// \brief Generic subscriptions for all topics.
    private: std::vector<std::shared_ptr<rclcpp::GenericSubscription>>
      subscriptions;

    /// \brief A rosbag2 writer.
    private: rosbag2_cpp::Writer bagWriter;

    /// \brief Whether the bag is open or not.
    private: bool bagOpen = false;

    /// \brief Mutex to protect the access to the bag.
    private: std::mutex mutex;

    /// \brief History of poses for jerk computation (stores last 4 samples).
    private: std::vector<geometry_msgs::msg::PoseStamped> poseHistory;

    /// \brief Computed linear jerk (x, y, z components in m/s^3).
    private: geometry_msgs::msg::Vector3 linearJerk;

    /// \brief Computed angular jerk (roll, pitch, yaw components in rad/s^3).
    private: geometry_msgs::msg::Vector3 angularJerk;

    /// \brief Time-weighted average linear jerk (x, y, z components in m/s^3).
    private: geometry_msgs::msg::Vector3 avgLinearJerk;

    /// \brief Time-weighted average angular jerk (roll, pitch, yaw in rad/s^3).
    private: geometry_msgs::msg::Vector3 avgAngularJerk;

    /// \brief Total elapsed time since last reset (seconds).
    private: double totalJerkTime = 0.0;

    /// \brief Accumulated weighted linear jerk (jerk * dt sum).
    private: geometry_msgs::msg::Vector3 accumLinearJerk;

    /// \brief Accumulated weighted angular jerk (jerk * dt sum).
    private: geometry_msgs::msg::Vector3 accumAngularJerk;

    /// \brief Current plug-port connection distance (meters).
    private: double plugPortDistance = 0.0;

    /// \brief Time-weighted average plug-port distance (meters).
    private: double avgPlugPortDistance = 0.0;

    /// \brief Accumulated weighted plug-port distance (distance * dt sum).
    private: double accumPlugPortDistance = 0.0;

    /// \brief Total elapsed time for plug-port distance computation (seconds).
    private: double totalPlugPortTime = 0.0;

    /// \brief Last timestamp for plug-port distance computation.
    private: double lastPlugPortStamp = -1.0;
  };

  // The Tier2 class as a node.
  class ScoringTier2Node : public rclcpp::Node
  {
    /// \brief Class constructor.
    /// \param[in] _yamlFile Path to a YAML config file.
    public: ScoringTier2Node(const std::string &_yamlFile);

    /// \brief The scoring.
    public: std::unique_ptr<ScoringTier2> score;
  };
}
#endif
