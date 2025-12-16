# Current state of the AIC Controller which includes the computed cartesian and joint commands

# Header
std_msgs/Header header

# Commanded states
geometry_msgs/Pose tool_pose # Computed tool pose
geometry_msgs/Twist tool_velocity # Computed tool velocity
trajectory_msgs/JointTrajectoryPoint joint_state # Computed joint state commands