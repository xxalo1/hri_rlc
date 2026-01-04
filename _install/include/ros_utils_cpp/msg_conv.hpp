#pragma once

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rlc_interfaces/msg/joint_effort_cmd.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ros_utils_cpp/types.hpp"

namespace ros_utils_cpp::msg_conv {

// Conversions never resize/allocate. All vectors on both sides must already have
// the correct size (preallocated in a cold path).
//
// Joint name fields are not copied to avoid allocations. If name vectors are
// non-empty, only sizes are validated.
//
// Optional message fields follow ROS semantics: if a destination message vector
// is empty, it stays empty and is not written (e.g., JointState velocity/effort).

[[nodiscard]] bool from_joint_state_msg(const sensor_msgs::msg::JointState& msg,
                                       JointStateMsgData& out) noexcept;
[[nodiscard]] bool to_joint_state_msg(const JointStateMsgData& data,
                                     sensor_msgs::msg::JointState& out) noexcept;

[[nodiscard]] bool from_joint_effort_cmd_msg(
    const rlc_interfaces::msg::JointEffortCmd& msg,
    JointEffortCmdMsgData& out) noexcept;
[[nodiscard]] bool to_joint_effort_cmd_msg(
    const JointEffortCmdMsgData& data,
    rlc_interfaces::msg::JointEffortCmd& out) noexcept;

[[nodiscard]] bool from_joint_trajectory_controller_state_msg(
    const control_msgs::msg::JointTrajectoryControllerState& msg,
    JointControllerStateMsgData& out) noexcept;
[[nodiscard]] bool to_joint_trajectory_controller_state_msg(
    const JointControllerStateMsgData& data,
    control_msgs::msg::JointTrajectoryControllerState& out) noexcept;

// Short aliases (match common naming used in Python tools).
[[nodiscard]] inline bool from_joint_ctrl_state_msg(
    const control_msgs::msg::JointTrajectoryControllerState& msg,
    JointControllerStateMsgData& out) noexcept {
  return from_joint_trajectory_controller_state_msg(msg, out);
}
[[nodiscard]] inline bool to_joint_ctrl_state_msg(
    const JointControllerStateMsgData& data,
    control_msgs::msg::JointTrajectoryControllerState& out) noexcept {
  return to_joint_trajectory_controller_state_msg(data, out);
}

}  // namespace ros_utils_cpp::msg_conv
