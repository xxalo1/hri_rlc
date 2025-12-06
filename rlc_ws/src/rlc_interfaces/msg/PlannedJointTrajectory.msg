# Planned joint-space trajectory with execution metadata.
#
# Emitted by planners and consumed by controllers/executors. The ID allows
# downstream services to reference or trigger the plan.
# Identifier for the planned trajectory.
string trajectory_id

# Full planned trajectory (waypoints + timing).
trajectory_msgs/JointTrajectory trajectory

# If true, execute immediately after planning instead of queueing.
bool execute_immediately

# Optional human-readable label (e.g., "quintic", "point").
string label
