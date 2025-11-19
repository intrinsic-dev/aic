# Command the robot to a pose.

# Header
std_msgs/Header header

# Note: These targets are used according to the trajectory_generation_mode specified below.
# Pose target.
geometry_msgs/Pose pose
# Velocity target.
geometry_msgs/Twist velocity
# Acceleration target.
geometry_msgs/Accel acceleration

# Impedance matrices for Cartesian impedance control.
float64[36] target_stiffness # 6x6 row-major matrix
float64[36] target_damping # 6x6 row-major matrix
float64[36] target_mass # 6x6 row-major matrix
geometry_msgs/Wrench feedforward_wrench_at_tip

# Feedback gains of force feedback control. These gains must be in [0, 0.95]
# to minimize risk of instability.
geometry_msgs/Wrench wrench_feedback_gains_at_tip

# There are three ways that input targets can be provided.
#   1. trajectory_generation_mode = VELOCITY
#      In this case, a target velocity must be provided. The target position
#      will be ignored.
#   2. trajectory_generation_mode = POSITION
#      In this case, a target position must be provided. The target velocity
#      will be ignored.
#   3. trajectory_generation_mode = POSITION_AND_VELOCITY
#      In this case, both target position and velocity will be used.
# Note, that UNSPECIFIED defaults to VELOCITY.
#
# The details of the trajectory generation are further determined by the
# config in TaskSettings.
uint8 TRAJECTORY_GENERATION_MODE_UNSPECIFIED = 0
uint8 TRAJECTORY_GENERATION_MODE_VELOCITY = 1
uint8 TRAJECTORY_GENERATION_MODE_POSITION = 2
uint8 TRAJECTORY_GENERATION_MODE_POSITION_AND_VELOCITY = 3
uint8 trajectory_generation_mode

# When provided, gives this action a SUGGESTED time to reach the given target
# at. It is not guaranteed that the target will be reached at this time as it
# may be infeasible given the pos/vel/acc/jerk limits. This value may also be
# ignored depending on the interpolation algorithm used by the aic_controller.
#
# When not provided, the behavior is determined by the interpolation
# algorithm and may pick a default or error out.
float64 time_to_target_seconds
