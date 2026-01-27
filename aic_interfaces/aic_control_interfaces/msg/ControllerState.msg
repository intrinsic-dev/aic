# Current state of the AIC Controller which includes the computed cartesian and joint commands

# Header
std_msgs/Header header

# Current tcp pose.
geometry_msgs/Pose tcp_pose
# Current tcp velocity.
geometry_msgs/Twist tcp_velocity
# Reference tcp pose.
geometry_msgs/Pose reference_tcp_pose
# Pose error from current to target tool pose in the form of (x,y,z,rx,ry,rz)
float64[6] tcp_error
# Reference joint state commands
trajectory_msgs/JointTrajectoryPoint reference_joint_state
