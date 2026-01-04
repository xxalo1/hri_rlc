#pragma once

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rlc_interfaces/msg/joint_effort_cmd.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rlc_utils/types.hpp"

namespace rlc_utils::msg_conv {

sensor_msgs::msg::JointState make_joint_state_msg(
    std::size_t n, double stamp_sec = 0.0,
    const std::vector<std::string>& names = {}, bool include_effort = true);

rlc_interfaces::msg::JointEffortCmd make_joint_effort_cmd_msg(
    std::size_t n, double stamp_sec = 0.0,
    const std::vector<std::string>& names = {});

control_msgs::msg::JointTrajectoryControllerState make_joint_ctrl_state_msg(
    std::size_t n, double stamp_sec = 0.0,
    const std::vector<std::string>& names = {},
    bool include_feedback_acceleration = false);

/**
 * @brief Copy a ROS JointState message into a preallocated data container.
 *
 * Copies joint arrays from @p msg into @p data without resizing @p data.
 *
 * Contract:
 * - `msg.position.size()` must equal `data.size()`.
 * - `msg.name`, `msg.velocity`, `msg.effort` may be empty; if non-empty, each
 * must have size `data.size()`.
 * - If `msg.velocity` is empty, `data.velocity` is set to zero.
 * - If `msg.effort` is empty, `data.effort` is set to zero.
 *
 * Sets `data.stamp_sec` from `msg.header.stamp`.
 * Joint names are not copied.
 *
 * @return true on success; false if any required/optional field violates the
 * size contract.
 *
 * @note Allocation-free; O(n) in the number of joints.
 * @warning noexcept: if a dependency throws (e.g., time conversion), the
 * program terminates.
 */
[[nodiscard]] bool from_joint_state_msg(const sensor_msgs::msg::JointState& msg,
                                        JointStateMsgData& out) noexcept;

[[nodiscard]] bool from_joint_effort_cmd_msg(
    const rlc_interfaces::msg::JointEffortCmd& msg,
    JointEffortCmdMsgData& out) noexcept;

[[nodiscard]] bool from_joint_ctrl_state_msg(
    const control_msgs::msg::JointTrajectoryControllerState& msg,
    JointControllerStateMsgData& out) noexcept;

[[nodiscard]] bool to_joint_state_msg(
    const JointStateMsgData& data, sensor_msgs::msg::JointState& msg) noexcept;

[[nodiscard]] bool to_joint_effort_cmd_msg(
    const JointEffortCmdMsgData& data,
    rlc_interfaces::msg::JointEffortCmd& out) noexcept;

[[nodiscard]] bool to_joint_ctrl_state_msg(
    const JointControllerStateMsgData& data,
    control_msgs::msg::JointTrajectoryControllerState& out) noexcept;

}  // namespace rlc_utils::msg_conv
