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
uint8 MODE_UNSPECIFIED = 0
uint8 MODE_VELOCITY = 1
uint8 MODE_POSITION = 2
uint8 mode
