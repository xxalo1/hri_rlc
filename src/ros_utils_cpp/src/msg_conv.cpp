#include "ros_utils_cpp/msg_conv.hpp"

#include <Eigen/Core>
#include <cstring>

#include "ros_utils_cpp/checks.hpp"
#include "ros_utils_cpp/time.hpp"

namespace ru = ros_utils_cpp;
namespace rtime = ros_utils_cpp::time;

namespace ros_utils_cpp::msg_conv {

bool from_joint_state_msg(const sensor_msgs::msg::JointState& msg,
                          JointStateMsgData& out) noexcept {
  const std::size_t n = out.size();

  if (!ros_utils_cpp::checks::size_is(msg.position, n)) return false;
  if (!ros_utils_cpp::checks::empty_or_size_is(msg.name, n)) return false;
  if (!ros_utils_cpp::checks::empty_or_size_is(msg.velocity, n)) return false;
  if (!ros_utils_cpp::checks::empty_or_size_is(msg.effort, n)) return false;

  out.stamp_sec = rtime::from_ros_time(msg.header.stamp);

  const std::size_t bytes = n * sizeof(double);

  std::memcpy(out.position.data(), msg.position.data(), bytes);

  if (!msg.velocity.empty()) {
    std::memcpy(out.velocity.data(), msg.velocity.data(), bytes);
  } else {
    out.velocity.setZero();
  }

  if (!msg.effort.empty()) {
    std::memcpy(out.effort.data(), msg.effort.data(), bytes);
  } else {
    out.effort.setZero();
  }

  return true;
}

bool to_joint_state_msg(const JointStateMsgData& data,
                        sensor_msgs::msg::JointState& out) noexcept {
  const auto n = ros_utils_cpp::checks::size_of(data.position);
  if (!ros_utils_cpp::checks::empty_or_size_is(out.name, n)) return false;
  if (!ros_utils_cpp::checks::size_is(out.position, n)) return false;
  if (!ros_utils_cpp::checks::empty_or_size_is(out.velocity, n)) return false;
  if (!ros_utils_cpp::checks::empty_or_size_is(out.effort, n)) return false;

  const bool write_velocity = !out.velocity.empty();
  const bool write_effort = !out.effort.empty();
  if (write_velocity && !ros_utils_cpp::checks::size_is(data.velocity, n)) {
    return false;
  }
  if (write_effort && !ros_utils_cpp::checks::size_is(data.effort, n)) {
    return false;
  }

  out.header.stamp = ros_utils_cpp::time::ToRosTime(data.stamp_sec);
  for (std::size_t i = 0; i < n; ++i) {
    const auto ii = static_cast<Eigen::Index>(i);
    out.position[i] = data.position[ii];
    if (write_velocity) out.velocity[i] = data.velocity[ii];
    if (write_effort) out.effort[i] = data.effort[ii];
  }

  return true;
}

bool from_joint_effort_cmd_msg(const rlc_interfaces::msg::JointEffortCmd& msg,
                               JointEffortCmdMsgData& out) noexcept {
  const auto n = ros_utils_cpp::checks::size_of(out.effort);
  if (!ros_utils_cpp::checks::size_is(msg.effort, n)) return false;
  if (!ros_utils_cpp::checks::empty_or_size_is(msg.name, n)) return false;

  out.stamp_sec = ros_utils_cpp::time::FromRosTime(msg.header.stamp);
  for (std::size_t i = 0; i < n; ++i) {
    out.effort[static_cast<Eigen::Index>(i)] = msg.effort[i];
  }

  return true;
}

bool to_joint_effort_cmd_msg(
    const JointEffortCmdMsgData& data,
    rlc_interfaces::msg::JointEffortCmd& out) noexcept {
  const auto n = ros_utils_cpp::checks::size_of(data.effort);
  if (!ros_utils_cpp::checks::size_is(out.effort, n)) return false;
  if (!ros_utils_cpp::checks::empty_or_size_is(out.name, n)) return false;

  out.header.stamp = ros_utils_cpp::time::ToRosTime(data.stamp_sec);
  for (std::size_t i = 0; i < n; ++i) {
    out.effort[i] = data.effort[static_cast<Eigen::Index>(i)];
  }

  return true;
}

bool from_joint_trajectory_controller_state_msg(
    const control_msgs::msg::JointTrajectoryControllerState& msg,
    JointControllerStateMsgData& out) noexcept {
  const auto n = ros_utils_cpp::checks::size_of(out.position);

  if (!ros_utils_cpp::checks::all_size_is(
          n, out.velocity, out.acceleration, out.position_des, out.velocity_des,
          out.acceleration_des, out.effort, out.position_error,
          out.velocity_error)) {
    return false;
  }

  if (!ros_utils_cpp::checks::empty_or_size_is(msg.joint_names, n))
    return false;
  if (!ros_utils_cpp::checks::all_empty_or_size_is(
          n, msg.reference.positions, msg.reference.velocities,
          msg.reference.accelerations, msg.feedback.positions,
          msg.feedback.velocities, msg.feedback.accelerations,
          msg.error.positions, msg.error.velocities, msg.output.effort)) {
    return false;
  }

  out.stamp_sec = ros_utils_cpp::time::FromRosTime(msg.header.stamp);
  const bool has_ref_pos = !msg.reference.positions.empty();
  const bool has_ref_vel = !msg.reference.velocities.empty();
  const bool has_ref_acc = !msg.reference.accelerations.empty();
  const bool has_fb_pos = !msg.feedback.positions.empty();
  const bool has_fb_vel = !msg.feedback.velocities.empty();
  const bool has_fb_acc = !msg.feedback.accelerations.empty();
  const bool has_error_pos = !msg.error.positions.empty();
  const bool has_error_vel = !msg.error.velocities.empty();
  const bool has_out_effort = !msg.output.effort.empty();

  for (std::size_t i = 0; i < n; ++i) {
    const auto ii = static_cast<Eigen::Index>(i);

    out.position_des[ii] = has_ref_pos ? msg.reference.positions[i] : 0.0;
    out.velocity_des[ii] = has_ref_vel ? msg.reference.velocities[i] : 0.0;
    out.acceleration_des[ii] =
        has_ref_acc ? msg.reference.accelerations[i] : 0.0;

    out.position[ii] = has_fb_pos ? msg.feedback.positions[i] : 0.0;
    out.velocity[ii] = has_fb_vel ? msg.feedback.velocities[i] : 0.0;
    out.acceleration[ii] = has_fb_acc ? msg.feedback.accelerations[i] : 0.0;

    out.effort[ii] = has_out_effort ? msg.output.effort[i] : 0.0;

    out.position_error[ii] = has_error_pos
                                 ? msg.error.positions[i]
                                 : (out.position_des[ii] - out.position[ii]);
    out.velocity_error[ii] = has_error_vel
                                 ? msg.error.velocities[i]
                                 : (out.velocity_des[ii] - out.velocity[ii]);
  }

  return true;
}

bool to_joint_trajectory_controller_state_msg(
    const JointControllerStateMsgData& data,
    control_msgs::msg::JointTrajectoryControllerState& out) noexcept {
  const auto n = ros_utils_cpp::checks::size_of(data.position);

  if (!ros_utils_cpp::checks::all_size_is(n, data.position_des, data.effort)) {
    return false;
  }
  if (!ros_utils_cpp::checks::empty_or_size_is(out.joint_names, n))
    return false;

  if (!ros_utils_cpp::checks::all_size_is(
          n, out.reference.positions, out.feedback.positions,
          out.error.positions, out.output.effort)) {
    return false;
  }
  if (!ros_utils_cpp::checks::all_empty_or_size_is(
          n, out.reference.velocities, out.reference.accelerations,
          out.feedback.velocities, out.feedback.accelerations,
          out.error.velocities)) {
    return false;
  }

  const bool write_ref_vel = !out.reference.velocities.empty();
  const bool write_ref_acc = !out.reference.accelerations.empty();
  const bool write_fb_vel = !out.feedback.velocities.empty();
  const bool write_fb_acc = !out.feedback.accelerations.empty();
  const bool write_error_vel = !out.error.velocities.empty();
  if (write_error_vel && !(write_ref_vel && write_fb_vel)) return false;

  if (write_ref_vel && !ros_utils_cpp::checks::size_is(data.velocity_des, n)) {
    return false;
  }
  if (write_ref_acc &&
      !ros_utils_cpp::checks::size_is(data.acceleration_des, n)) {
    return false;
  }
  if (write_fb_vel && !ros_utils_cpp::checks::size_is(data.velocity, n)) {
    return false;
  }
  if (write_fb_acc && !ros_utils_cpp::checks::size_is(data.acceleration, n)) {
    return false;
  }

  out.header.stamp = ros_utils_cpp::time::ToRosTime(data.stamp_sec);

  for (std::size_t i = 0; i < n; ++i) {
    const auto ii = static_cast<Eigen::Index>(i);

    out.reference.positions[i] = data.position_des[ii];
    if (write_ref_vel) out.reference.velocities[i] = data.velocity_des[ii];
    if (write_ref_acc)
      out.reference.accelerations[i] = data.acceleration_des[ii];

    out.feedback.positions[i] = data.position[ii];
    if (write_fb_vel) out.feedback.velocities[i] = data.velocity[ii];
    if (write_fb_acc) out.feedback.accelerations[i] = data.acceleration[ii];

    out.error.positions[i] = data.position_des[ii] - data.position[ii];
    if (write_error_vel)
      out.error.velocities[i] = data.velocity_des[ii] - data.velocity[ii];

    out.output.effort[i] = data.effort[ii];
  }

  return true;
}

}  // namespace ros_utils_cpp::msg_conv
