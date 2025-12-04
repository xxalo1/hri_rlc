# Simulated joint state mirroring sensor_msgs/JointState with explicit sim time.
#
# Published by the simulator to provide deterministic time stamps alongside the
# joint arrays. All arrays must align index-for-index with `name`.
std_msgs/Header header  # Stamp uses simulation clock; frame_id optional.

# Simulation time in seconds for the reported state.
float64 sim_time

# Joint names for which state is reported.
string[] name

# Joint positions (rad), ordered to match `name`.
float64[] position

# Joint velocities (rad/s), ordered to match `name`.
float64[] velocity

# joint accelerations (rad/s^2), ordered to match `name`.
float64[] acceleration

# Joint efforts/torques (Nm), ordered to match `name`.
float64[] effort
