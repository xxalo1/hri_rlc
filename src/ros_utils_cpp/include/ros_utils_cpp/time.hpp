#pragma once

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace ros_utils_cpp::time {

double from_ros_time(const builtin_interfaces::msg::Time& t) noexcept;
double from_ros_duration(const builtin_interfaces::msg::Duration& d) noexcept;

builtin_interfaces::msg::Time to_ros_time(double sec) noexcept;
builtin_interfaces::msg::Duration to_ros_duration(double sec) noexcept;

}  // namespace ros_utils_cpp::time