# Joint state for simulated robots, carrying both ROS time and simulation time.

# Standard ROS header. Typically use ROS time (wall or /use_sim_time) here.
std_msgs/Header header

# Simulation time in seconds (float64). This is your env's "t".
float64 sim_time

# Joint names.
string[] name

# Joint positions (rad).
float64[] position

# Joint velocities (rad/s).
float64[] velocity

# Joint efforts/torques (Nm).
float64[] effort
