# Command torques (or efforts) for a set of joints.

std_msgs/Header header

# Joint names this command applies to. Must be same length as effort.
string[] name

# Commanded effort/torque for each joint (Nm).
float64[] effort
