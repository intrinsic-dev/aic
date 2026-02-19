# There are two types of target modes.
#   1. target_mode = MODE_CARTESIAN
#      The controller will process Cartesian targets of type 'MotionUpdate' and 
#      ignore other target types
#   2. target_mode = MODE_JOINT
#      The controller will process joint targets of type 'JointMotionUpdate' and 
#      ignore other target types.
uint8 MODE_UNSPECIFIED = 0 
uint8 MODE_CARTESIAN = 1   
uint8 MODE_JOINT = 2       
uint8 mode
