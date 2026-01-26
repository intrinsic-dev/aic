# Command the robot to a target cartesian state.

# Header
std_msgs/Header header

# Note: These targets are used according to the trajectory_generation_mode specified below.
# Pose target.
geometry_msgs/Pose pose
# Velocity target.
geometry_msgs/Twist velocity

# Impedance matrices for Cartesian impedance control.
float64[36] target_stiffness # 6x6 row-major matrix
float64[36] target_damping # 6x6 row-major matrix
geometry_msgs/Wrench feedforward_wrench_at_tip

# [OPTIONAL] Feedback gains of force feedback control. These gains must be in [0, 0.95]
# to minimize risk of instability.
geometry_msgs/Wrench wrench_feedback_gains_at_tip

# The mode of trajectory generation.
TrajectoryGenerationMode trajectory_generation_mode

