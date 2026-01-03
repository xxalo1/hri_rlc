#pragma once

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rlc_interfaces/msg/joint_effort_cmd.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Core>

namespace ros_utils_cpp::msg_conv {

// Reads JointState into preallocated vectors (no resize, no allocations).
// Returns false if sizes mismatch or required fields are missing.
bool ReadJointStateNoAlloc(const sensor_msgs::msg::JointState& msg,
                           Eigen::Ref<Eigen::VectorXd> q_out,
                           Eigen::Ref<Eigen::VectorXd> qd_out,
                           double* stamp_sec_out) noexcept;

// Writes effort command into a preallocated message (no resize, no allocations).
// Assumes out->name is already populated (set once during I/O configuration).
bool WriteJointEffortCmdNoAlloc(double stamp_sec,
                                const Eigen::Ref<const Eigen::VectorXd>& effort,
                                rlc_interfaces::msg::JointEffortCmd* out) noexcept;

// Writes controller state into a preallocated message (no resize, no allocations).
// Assumes out->joint_names is already populated (set once during I/O configuration).
bool WriteJointCtrlStateNoAlloc(
    double stamp_sec, const Eigen::Ref<const Eigen::VectorXd>& tau,
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& qd,
    const Eigen::Ref<const Eigen::VectorXd>& q_des,
    const Eigen::Ref<const Eigen::VectorXd>& qd_des,
    const Eigen::Ref<const Eigen::VectorXd>& qdd_des,
    control_msgs::msg::JointTrajectoryControllerState* out) noexcept;

}  // namespace ros_utils_cpp::msg_conv

