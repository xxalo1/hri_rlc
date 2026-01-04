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

  if (!ru::checks::all_size_is(n, out.position, out.velocity, out.effort)) {
    return false;
  }
  if (!ru::checks::size_is(msg.position, n)) return false;
  if (!ru::checks::empty_or_size_is(msg.name, n)) return false;
  if (!ru::checks::empty_or_size_is(msg.velocity, n)) return false;
  if (!ru::checks::empty_or_size_is(msg.effort, n)) return false;

  out.stamp_sec = rtime::from_ros_time(msg.header.stamp);

  const std::size_t bytes = n * sizeof(double);

  if (bytes != 0) {
    std::memcpy(out.position.data(), msg.position.data(), bytes);
  }

  if (!msg.velocity.empty()) {
    if (bytes != 0) {
      std::memcpy(out.velocity.data(), msg.velocity.data(), bytes);
    }
  } else {
    out.velocity.setZero();
  }

  if (!msg.effort.empty()) {
    if (bytes != 0) {
      std::memcpy(out.effort.data(), msg.effort.data(), bytes);
    }
  } else {
    out.effort.setZero();
  }

  return true;
}

bool to_joint_state_msg(const JointStateMsgData& data,
                        sensor_msgs::msg::JointState& out) noexcept {
  const std::size_t n = data.size();
  if (!ru::checks::empty_or_size_is(out.name, n)) return false;
  if (!ru::checks::size_is(out.position, n)) return false;
  if (!ru::checks::empty_or_size_is(out.velocity, n)) return false;
  if (!ru::checks::empty_or_size_is(out.effort, n)) return false;

  const bool write_velocity = !out.velocity.empty();
  const bool write_effort = !out.effort.empty();
  if (write_velocity && !ru::checks::size_is(data.velocity, n)) return false;
  if (write_effort && !ru::checks::size_is(data.effort, n)) return false;

  out.header.stamp = rtime::to_ros_time(data.stamp_sec);

  const std::size_t bytes = n * sizeof(double);
  if (bytes != 0) {
    std::memcpy(out.position.data(), data.position.data(), bytes);
  }
  if (write_velocity && bytes != 0) {
    std::memcpy(out.velocity.data(), data.velocity.data(), bytes);
  }
  if (write_effort && bytes != 0) {
    std::memcpy(out.effort.data(), data.effort.data(), bytes);
  }

  return true;
}

bool from_joint_effort_cmd_msg(const rlc_interfaces::msg::JointEffortCmd& msg,
                               JointEffortCmdMsgData& out) noexcept {
  const std::size_t n = out.size();
  if (!ru::checks::size_is(msg.effort, n)) return false;
  if (!ru::checks::empty_or_size_is(msg.name, n)) return false;

  out.stamp_sec = rtime::from_ros_time(msg.header.stamp);
  const std::size_t bytes = n * sizeof(double);
  if (bytes != 0) {
    std::memcpy(out.effort.data(), msg.effort.data(), bytes);
  }

  return true;
}

bool to_joint_effort_cmd_msg(
    const JointEffortCmdMsgData& data,
    rlc_interfaces::msg::JointEffortCmd& out) noexcept {
  const std::size_t n = data.size();
  if (!ru::checks::size_is(out.effort, n)) return false;
  if (!ru::checks::empty_or_size_is(out.name, n)) return false;

  out.header.stamp = rtime::to_ros_time(data.stamp_sec);
  const std::size_t bytes = n * sizeof(double);
  if (bytes != 0) {
    std::memcpy(out.effort.data(), data.effort.data(), bytes);
  }

  return true;
}

bool from_joint_trajectory_controller_state_msg(
    const control_msgs::msg::JointTrajectoryControllerState& msg,
    JointControllerStateMsgData& out) noexcept {
  const std::size_t n = out.size();

  if (!ru::checks::all_size_is(n, out.velocity, out.acceleration, out.position_des,
                              out.velocity_des, out.acceleration_des, out.effort,
                              out.position_error, out.velocity_error)) {
    return false;
  }

  if (!ru::checks::empty_or_size_is(msg.joint_names, n)) return false;
  if (!ru::checks::all_empty_or_size_is(
          n, msg.reference.positions, msg.reference.velocities,
          msg.reference.accelerations, msg.feedback.positions,
          msg.feedback.velocities, msg.feedback.accelerations, msg.error.positions,
          msg.error.velocities, msg.output.effort)) {
    return false;
  }

  out.stamp_sec = rtime::from_ros_time(msg.header.stamp);
  const std::size_t bytes = n * sizeof(double);

  auto copy_or_zero = [bytes](Eigen::VectorXd& dst,
                              const std::vector<double>& src) noexcept {
    if (src.empty()) {
      dst.setZero();
      return;
    }
    if (bytes != 0) {
      std::memcpy(dst.data(), src.data(), bytes);
    }
  };

  copy_or_zero(out.position_des, msg.reference.positions);
  copy_or_zero(out.velocity_des, msg.reference.velocities);
  copy_or_zero(out.acceleration_des, msg.reference.accelerations);

  copy_or_zero(out.position, msg.feedback.positions);
  copy_or_zero(out.velocity, msg.feedback.velocities);
  copy_or_zero(out.acceleration, msg.feedback.accelerations);

  copy_or_zero(out.effort, msg.output.effort);

  if (msg.error.positions.empty()) {
    out.position_error.noalias() = out.position_des - out.position;
  } else {
    copy_or_zero(out.position_error, msg.error.positions);
  }

  if (msg.error.velocities.empty()) {
    out.velocity_error.noalias() = out.velocity_des - out.velocity;
  } else {
    copy_or_zero(out.velocity_error, msg.error.velocities);
  }

  return true;
}

bool to_joint_trajectory_controller_state_msg(
    const JointControllerStateMsgData& data,
    control_msgs::msg::JointTrajectoryControllerState& out) noexcept {
  const std::size_t n = data.size();

  if (!ru::checks::all_size_is(n, data.position_des, data.effort)) {
    return false;
  }
  if (!ru::checks::empty_or_size_is(out.joint_names, n)) return false;

  if (!ru::checks::all_size_is(n, out.reference.positions, out.feedback.positions,
                              out.error.positions, out.output.effort)) {
    return false;
  }
  if (!ru::checks::all_empty_or_size_is(n, out.reference.velocities,
                                       out.reference.accelerations,
                                       out.feedback.velocities,
                                       out.feedback.accelerations,
                                       out.error.velocities)) {
    return false;
  }

  const bool write_ref_vel = !out.reference.velocities.empty();
  const bool write_ref_acc = !out.reference.accelerations.empty();
  const bool write_fb_vel = !out.feedback.velocities.empty();
  const bool write_fb_acc = !out.feedback.accelerations.empty();
  const bool write_error_vel = !out.error.velocities.empty();
  if (write_error_vel && !(write_ref_vel && write_fb_vel)) return false;

  if (write_ref_vel && !ru::checks::size_is(data.velocity_des, n)) return false;
  if (write_ref_acc && !ru::checks::size_is(data.acceleration_des, n)) return false;
  if (write_fb_vel && !ru::checks::size_is(data.velocity, n)) return false;
  if (write_fb_acc && !ru::checks::size_is(data.acceleration, n)) return false;

  out.header.stamp = rtime::to_ros_time(data.stamp_sec);
  const std::size_t bytes = n * sizeof(double);

  if (bytes != 0) {
    std::memcpy(out.reference.positions.data(), data.position_des.data(), bytes);
    std::memcpy(out.feedback.positions.data(), data.position.data(), bytes);
    std::memcpy(out.output.effort.data(), data.effort.data(), bytes);
  }

  if (write_ref_vel && bytes != 0) {
    std::memcpy(out.reference.velocities.data(), data.velocity_des.data(), bytes);
  }
  if (write_ref_acc && bytes != 0) {
    std::memcpy(out.reference.accelerations.data(), data.acceleration_des.data(),
                bytes);
  }
  if (write_fb_vel && bytes != 0) {
    std::memcpy(out.feedback.velocities.data(), data.velocity.data(), bytes);
  }
  if (write_fb_acc && bytes != 0) {
    std::memcpy(out.feedback.accelerations.data(), data.acceleration.data(), bytes);
  }

  if (n != 0) {
    auto out_error_pos = Eigen::Map<Eigen::VectorXd>(
        out.error.positions.data(), static_cast<Eigen::Index>(n));
    out_error_pos.noalias() = data.position_des - data.position;

    if (write_error_vel) {
      auto out_error_vel = Eigen::Map<Eigen::VectorXd>(
          out.error.velocities.data(), static_cast<Eigen::Index>(n));
      out_error_vel.noalias() = data.velocity_des - data.velocity;
    }
  }

  return true;
}

}  // namespace ros_utils_cpp::msg_conv
