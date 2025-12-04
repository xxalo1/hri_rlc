# rlc_interfaces/msg/EndEffectorTrajectory.msg
# End-effector trajectory in a given reference frame.

# Standard header
# - stamp: time at which the trajectory was generated
# - frame_id: reference frame (e.g. "world" or "base_link")
std_msgs/Header header

# Time from the start of the trajectory for each waypoint.
# time_from_start[i] corresponds to poses[i]
duration[] time_from_start

# Pose of the end-effector at each waypoint, in header.frame_id
geometry_msgs/Pose[] poses
