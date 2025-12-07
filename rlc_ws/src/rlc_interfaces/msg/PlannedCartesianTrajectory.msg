# Planned Cartesian-space trajectory.

# Identifier for the planned trajectory.
string trajectory_id

# Full planned trajectory (waypoints + timing).
trajectory_msgs/MultiDOFJointTrajectory trajectory

# If true, execute immediately after planning instead of queueing.
bool execute_immediately

# Optional human-readable label (e.g., "quintic", "point").
string label

# True if this EE traj was projected from a joint traj
bool derived_from_joint