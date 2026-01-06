# Current state of the AIC Controller which includes the computed cartesian and joint commands

# Header
std_msgs/Header header

# Commanded states
geometry_msgs/Pose tcp_pose # Computed tcp pose
geometry_msgs/Twist tcp_velocity # Computed tcp velocity
float64[6] tcp_error # Pose error from current to target tool pose in the form of (x,y,z,rx,ry,rz)
trajectory_msgs/JointTrajectoryPoint joint_state # Computed joint state commands