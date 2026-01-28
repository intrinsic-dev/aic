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

