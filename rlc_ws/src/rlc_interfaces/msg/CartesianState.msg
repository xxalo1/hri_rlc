# Cartesian state of a frame combining pose and spatial velocity.
#
# Used to describe the end-effector or any tracked frame relative to a chosen
# reference frame. Wrench is intentionally omitted to keep the interface light.
std_msgs/Header header  # Frame_id names the reference frame for pose and twist.

# Pose of the frame in the reference frame.
geometry_msgs/Pose pose

# Spatial velocity (linear + angular) expressed in the same reference frame.
geometry_msgs/Twist twist
