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

#ifndef AIC_ENGINE_HPP_
#define AIC_ENGINE_HPP_

#include <aic_engine_interfaces/srv/reset_joints.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

#include "aic_control_interfaces/msg/joint_motion_update.hpp"
#include "aic_control_interfaces/msg/motion_update.hpp"
#include "aic_control_interfaces/msg/trajectory_generation_mode.hpp"
#include "aic_scoring/ScoringTier2.hh"
#include "aic_scoring/TierScore.hh"
#include "aic_task_interfaces/action/insert_cable.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "simulation_interfaces/srv/delete_entity.hpp"
#include "simulation_interfaces/srv/spawn_entity.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "yaml-cpp/yaml.h"

//==============================================================================
namespace aic {

using DeleteEntitySrv = simulation_interfaces::srv::DeleteEntity;
using InsertCableAction = aic_task_interfaces::action::InsertCable;
using InsertCableGoalHandle =
    rclcpp_action::ServerGoalHandle<InsertCableAction>;
using JointMotionUpdateMsg = aic_control_interfaces::msg::JointMotionUpdate;
using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using MotionUpdateMsg = aic_control_interfaces::msg::MotionUpdate;
using ResetJointsSrv = aic_engine_interfaces::srv::ResetJoints;
using SpawnEntitySrv = simulation_interfaces::srv::SpawnEntity;
using SwitchControllerSrv = controller_manager_msgs::srv::SwitchController;
using Task = aic_task_interfaces::msg::Task;
using TrajectoryGenerationMode =
    aic_control_interfaces::msg::TrajectoryGenerationMode;
using WrenchStampedMsg = geometry_msgs::msg::WrenchStamped;

//==============================================================================
enum class EngineState : uint8_t {
  Uninitialized = 0,
  Initialized,
  Running,
  Error,
  Completed
};

//==============================================================================
// For each trial, track its state.
// States progress from Uninitialized -> EndpointsReady -> SimulatorReady
// -> ScoringReady -> TasksExecuting -> AllTasksCompleted
// Uninitialized: Trial has not started.
// ModelReady: Participant model node is available and conforms to challenge
// requirements.
// EndpointsReady: Required nodes are up and running.
// SimulatorReady: Simulator is ready with the task board and cables spawned.
// ScoringReady: Scoring system is ready to track performance.
// TasksExecuting: Tasks are being executed.
// AllTasksCompleted: All tasks has been completed successfully or time limit
// reached.
enum class TrialState : uint8_t {
  Uninitialized = 0,
  ModelReady,
  EndpointsReady,
  SimulatorReady,
  ScoringReady,
  TasksExecuting,
  AllTasksCompleted
};

//==============================================================================
// For each task, track its state.
// States progress from Uninitialized -> TaskRequested -> TaskStarted ->
// TaskCompleted for successful runs.
// Uninitialized: Task has not started.
// TaskRequest: Task goal has been sent.
// TaskStarted: Task has been started executing.
// TaskCompleted: Task has been completed successfully.
// TaskRejected: Task was rejected.
// TaskFailed: Task has failed.
// TimeLimitExceeded: Task's configured time limit was exceeded.
enum class TaskState : uint8_t {
  Uninitialized = 0,
  TaskRequested,
  TaskStarted,
  TaskCompleted,
  TaskRejected,
  TaskFailed,
  TimeLimitExceeded,
};

//==============================================================================
struct TaskAttempt {
  // Constructors.
  TaskAttempt(const std::string& id);

  std::string id;
  std::optional<rclcpp::Time> time_started;
  std::optional<rclcpp::Time> time_completed;
  bool success;
  TaskState state;
};

//==============================================================================
struct Trial {
  // Constructor.
  // Throws std::runtime_error error if config is invalid.
  Trial(const std::string& id, YAML::Node config);

  std::string id;
  std::vector<std::string> spawned_entities;
  YAML::Node config;
  std::vector<Task> tasks;
  std::vector<TaskAttempt> attempts;
  TrialState state;
};

//==============================================================================
struct TrialScore {
  aic_scoring::Tier1Score tier_1;
  aic_scoring::Tier2Score tier_2;
  aic_scoring::Tier3Score tier_3;

  TrialScore()
      : tier_1(0),
        tier_2("Task execution failed"),
        tier_3(0, "Task execution failed") {}

  void tier_1_success() { tier_1 = aic_scoring::Tier1Score(1); }

  double total_score() const {
    return tier_1.total_score() + tier_2.total_score() + tier_3.total_score();
  }
};

struct Score {
  // Intentionally alphabetically sorted, trial_id -> trial_score
  std::map<std::string, TrialScore> breakdown;

  /// \brief Serializes the score into a YAML node for logging.
  /// \return The resulting YAML node with serialized data.
  YAML::Node serialize() const;

  /// \brief Computes the total score from the score breakdown.
  double calculate_total_score() const;
};

//==============================================================================
// Ensure rclcpp::init has been called before creating an instance.
class Engine {
 public:
  /// \brief Constructor.
  Engine(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// \brief Destructor.
  ~Engine();

  /// \brief Start the engine.
  void start();

 private:
  // Initializes the engine.
  EngineState initialize();

  /// \brief Run the engine.
  EngineState run();

  /// \brief Handle the logic for a given trial.
  /// \param[in] trial The trial to handle.
  /// \return The resulting score of the trial after handling.
  TrialScore handle_trial(Trial& trial);

  /// \brief Reset internal and simulator states after a trial is completed.
  /// \param[in] trial The trial currently being ran
  void reset_after_trial(const Trial& trial);

  /// \brief Reset robot back to home joint positions.
  bool home_robot();

  /// \brief Check if the participant model is ready. As per challenge
  /// requirements. See challenge_rules.md for details. \return True if the
  /// model is ready, false otherwise.
  bool check_model();

  /// \brief Check if required endpoints are available.
  /// \return True if all required endpoints are available, false otherwise.
  bool check_endpoints();

  /// \brief Check if the simulator is ready.
  /// \param[in] trial The trial currently being ran
  /// \return True if the simulator is ready, false otherwise.
  bool ready_simulator(Trial& trial);

  /// \brief Check if the scoring system is ready.
  /// \param[in] trial The trial currently being ran
  /// \return True if the scoring system is ready, false otherwise.
  bool ready_scoring(const Trial& trial);

  /// \brief Check if tasks were started successfully.
  /// \param[in] trial The trial currently being ran
  /// \return True if tasks were started successfully, false otherwise.
  bool tasks_started(Trial& trial);

  /// \brief Check if all tasks have been completed successfully.
  /// \param[in] trial The trial currently being ran
  /// \return True if tasks were completed successfully, false otherwise.
  bool tasks_completed_successfully(const Trial& trial);

  /// \brief Spawn an entity in Gazebo.
  /// \param[in] trial The trial currently being ran
  /// \param[in] entity_name Name of the entity to spawn
  /// \param[in] filepath Path to the xacro file of the entity
  /// \param[in] x X position
  /// \param[in] y Y position
  /// \param[in] z Z position
  /// \param[in] roll Roll orientation (radians)
  /// \param[in] pitch Pitch orientation (radians)
  /// \param[in] yaw Yaw orientation (radians)
  /// \return True if spawning succeeded, false otherwise
  bool spawn_entity(Trial& trial, std::string entity_name, std::string filepath,
                    double x, double y, double z, double roll, double pitch,
                    double yaw);

  /// @brief Check if the robot was commanded to move by the model node.
  /// @return True if the robot was commanded to move, false otherwise.
  bool model_node_moved_robot();

  /// @brief Check if the model is in the unconfigured state together with other
  /// expectations in this state.
  /// @return True if the model is unconfigured, false otherwise.
  bool model_node_is_unconfigured();

  /// @brief Trigger a state transition for the lifecycle node.
  /// \param[in] transition The transition to trigger as per
  /// lifecycle_msgs::msg::Transition enum definition.
  /// @return True if transition succeeded, false otherwise.
  bool transition_model_lifecycle_node(const uint8_t transition);

  /// @brief Configure the model node and check expectations in the configured
  /// state as per challenge requirements.
  /// @return True if configuration succeeded, false otherwise.
  bool configure_model_node();

  /// @brief Activate the model node to transition from configured to active
  /// state.
  /// @return True if activation succeeded, false otherwise.
  bool activate_model_node();

  /// @brief Deactivate the model node to transition from active to configured
  /// state.
  /// @return True if deactivation succeeded, false otherwise.
  bool deactivate_model_node();

  /// @brief Cleanup the model node to transition from inactive to
  /// unconfigured state.
  /// @return True if cleanup succeeded, false otherwise.
  bool cleanup_model_node();

  /// @brief Shutdown the model node to transition from unconfigured to
  /// shutdown state.
  /// @return True if shutdown succeeded, false otherwise.
  bool shutdown_model_node();

  /// @brief Stop the bag recording and score the current trial
  /// @param[in] A reference to the current trial score to update.
  void score_trial(TrialScore& trial);

  /// @brief Scores the current run, writing its result to a YAML file.
  /// \param[in] The score to serialize and write.
  void score_run(const Score& score);

  // Strings.
  // Name of the aic_adapter node for lifecycle transitions.
  std::string adapter_node_name_;
  // Name of the participant's model node for lifecycle transitions.
  std::string model_node_name_;
  // Name of the service to get the lifecycle state of the model node.
  std::string model_get_state_service_name_;
  // Name of the service to change the lifecycle state of the model node.
  std::string model_change_state_service_name_;

  // Internal ROS 2 node.
  rclcpp::Node::SharedPtr node_;
  // Subscriptions.
  rclcpp::Subscription<JointMotionUpdateMsg>::SharedPtr
      joint_motion_update_sub_;
  rclcpp::Subscription<MotionUpdateMsg>::SharedPtr motion_update_sub_;

  // Subscription messages.
  JointMotionUpdateMsg::ConstSharedPtr last_joint_motion_update_msg_;
  MotionUpdateMsg::ConstSharedPtr last_motion_update_msg_;

  // Action clients.
  rclcpp_action::Client<InsertCableAction>::SharedPtr
      insert_cable_action_client_;

  // Service clients.
  rclcpp::Client<SpawnEntitySrv>::SharedPtr spawn_entity_client_;
  rclcpp::Client<DeleteEntitySrv>::SharedPtr delete_entity_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr
      model_get_state_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr
      model_change_state_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
      switch_controller_client_;
  rclcpp::Client<ResetJointsSrv>::SharedPtr reset_joints_client_;

  // TF
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Task config.
  YAML::Node config_;

  // All trials parsed from config.
  std::vector<std::pair<std::string, Trial>> trials_;

  // Variable to track first trial as want to configure model only once.
  bool is_first_trial_;

  // Thread to spin ROS 2 node.
  std::thread spin_thread_;

  // Engine state.
  EngineState engine_state_;

  // Whether to publish ground truth data for scoring.
  bool ground_truth_;

  // Parameters to skip states for testing purposes.
  bool skip_model_ready_;
  bool skip_ready_simulator_;

  // Whether the participant model has been discovered and readied.
  bool model_discovered_;

  // Pre-built messages for homing robot (built once in initialize())
  JointMotionUpdateMsg home_joint_msg_;
  std::shared_ptr<ResetJointsSrv::Request> home_reset_joints_request_;

  // Scoring tier 2 instance.
  std::unique_ptr<aic_scoring::ScoringTier2> scoring_tier2_;

  // Output directory for scoring.
  std::string scoring_output_dir_;
};

}  // namespace aic

#endif  // AIC_ENGINE_HPP_
