#pragma once

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace ros_utils_cpp::time {

double FromRosTime(const builtin_interfaces::msg::Time& t) noexcept;
double FromRosDuration(const builtin_interfaces::msg::Duration& d) noexcept;

builtin_interfaces::msg::Time ToRosTime(double sec) noexcept;
builtin_interfaces::msg::Duration ToRosDuration(double sec) noexcept;

}  // namespace ros_utils_cpp::time