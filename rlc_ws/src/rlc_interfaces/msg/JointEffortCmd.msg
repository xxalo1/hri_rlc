# Torque command to apply to a set of joints.
#
# Used by controllers to send instantaneous effort values to the simulator or
# hardware. One effort value is paired with each joint name.
std_msgs/Header header  # Timestamp should align with the controller clock.

# Joint names in the order the commands are provided.
string[] name

# Commanded joint efforts/torques (Nm), matching the order of `name`.
float64[] effort

# time sim
float64 time_sim
