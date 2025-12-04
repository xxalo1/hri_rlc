# rlc_interfaces/msg/RobotFrames.msg
# DH or kinematic frames expressed in a common world frame.

# Standard header
# - stamp: time at which these frames are valid
# - frame_id: world frame name (e.g. "world", "base_link")
std_msgs/Header header

# Optional explicit world frame name (often same as header.frame_id)
string world_frame

# Names of each frame corresponding to each pose
# e.g. ["base_link", "link1", "link2", ...]
string[] frame_names

# Pose of each frame in the world frame
# poses[i] corresponds to frame_names[i]
geometry_msgs/Pose[] poses
