# FlipCommand.msg

# Axis of rotation in the AUV body frame
uint8 AXIS_X=0
uint8 AXIS_Y=1
uint8 AXIS_Z=2
uint8 axis

# Number of full 360-degree flips
uint32 count

# Direction according to the right-hand rule:
# +1 = positive rotation about selected axis
# -1 = negative rotation about selected axis
int8 direction

# Duration of each individual 360-degree flip, in seconds
float64 flip_duration