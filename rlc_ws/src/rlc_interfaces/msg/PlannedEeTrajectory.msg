# rlc_interfaces/msg/PlannedEeTrajectory.msg

string trajectory_id       # same id as the joint-space plan, if there is one
string label               # "quintic", "cartesian", etc.
bool   derived_from_joint  # True if this EE traj was projected from a joint traj

trajectory_msgs/MultiDOFJointTrajectory trajectory
