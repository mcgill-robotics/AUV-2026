# auv_msgs/msg/AttitudeReference.msg

# orientation reference. Expressed from the inertial frame to the body frame. 
geometry_msgs/Quaternion orientation 

# Desired angular velocity in the AUV/body frame, rad/s.
# Defaults to zero if not set by the publisher.
geometry_msgs/Vector3 angular_velocity