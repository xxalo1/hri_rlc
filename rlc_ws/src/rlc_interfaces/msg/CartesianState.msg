# Cartesian state of a frame (pose + twist + wrench).

std_msgs/Header header

# Pose of the frame in some reference frame.
geometry_msgs/Pose pose

# Spatial velocity (linear + angular), in the same frame.
geometry_msgs/Twist twist