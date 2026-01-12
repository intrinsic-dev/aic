# Command the robot to a target joint state.

# The target joint configuration including set points for positions,
# velocities, accelerations, and efforts.
trajectory_msgs/JointTrajectoryPoint target_state

# Stiffness and damping both evolve from a current value toward these target
# ones. This avoids sudden jumps when the values are changed.
# Specify for each joint.
float64[] target_stiffness
float64[] target_damping

# [OPTIONAL] The mode of trajectory generation.
TrajectoryGenerationMode trajectory_generation_mode

float64[] target_feedforward_torque

# When provided, gives this action a SUGGESTED time to reach the given target
# at. It is not guaranteed that the target will be reached at this time as it
# may be infeasible given the pos/vel/acc/jerk limits. This value may also be
# ignored depending on the interpolation algorithm used by the aic_controller.
#
# When not provided, the behavior is determined by the interpolation
# algorithm and may pick a default or error out.
float64 time_to_target_seconds
