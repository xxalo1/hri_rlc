std_msgs/Header header

# Joint names in the order the commands are provided.
string[] name

# Commanded joint efforts/torques (Nm), matching the order of `name`.
float64[] effort