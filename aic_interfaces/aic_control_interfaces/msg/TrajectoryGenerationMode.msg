# There are two ways that input targets can be provided for the MotionUpdate message.
#   1. mode = MODE_VELOCITY
#      In this case, a target velocity must be provided. The target pose
#      will be ignored.
#   2. mode = MODE_POSITION
#      In this case, a target pose must be provided. The target velocity
#      will be ignored.
# Notes that messages with mode set to MODE_UNSPECIFIED will be ignored.
uint8 MODE_UNSPECIFIED = 0
uint8 MODE_VELOCITY = 1
uint8 MODE_POSITION = 2
uint8 mode
