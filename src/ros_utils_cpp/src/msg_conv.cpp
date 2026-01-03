#include "ros_utils_cpp/msg_conv.hpp"

#include "ros_utils_cpp/time.hpp"

namespace ros_utils_cpp::msg_conv {

bool ReadJointStateNoAlloc(const sensor_msgs::msg::JointState& msg,
                           Eigen::Ref<Eigen::VectorXd> q_out,
                           Eigen::Ref<Eigen::VectorXd> qd_out,
                           double* stamp_sec_out) noexcept {
  const auto n = static_cast<std::size_t>(q_out.size());
  if (qd_out.size() != q_out.size()) return false;
  if (msg.position.size() != n) return false;
  if (msg.velocity.size() != n) return false;
  if (!stamp_sec_out) return false;

  for (std::size_t i = 0; i < n; ++i) {
    q_out[static_cast<int>(i)] = msg.position[i];
    qd_out[static_cast<int>(i)] = msg.velocity[i];
  }

  *stamp_sec_out = ros_utils_cpp::time::ToSec(msg.header.stamp);
  return true;
}

bool WriteJointEffortCmdNoAlloc(double stamp_sec,
                                const Eigen::Ref<const Eigen::VectorXd>& effort,
                                rlc_interfaces::msg::JointEffortCmd* out) noexcept {
  if (!out) return false;
  const auto n = static_cast<std::size_t>(effort.size());
  if (out->effort.size() != n) return false;
  if (!out->name.empty() && out->name.size() != n) return false;

  out->header.stamp = ros_utils_cpp::time::ToRosTime(stamp_sec);
  for (std::size_t i = 0; i < n; ++i) {
    out->effort[i] = effort[static_cast<int>(i)];
  }
  return true;
}

bool WriteJointCtrlStateNoAlloc(
    double stamp_sec, const Eigen::Ref<const Eigen::VectorXd>& tau,
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& qd,
    const Eigen::Ref<const Eigen::VectorXd>& q_des,
    const Eigen::Ref<const Eigen::VectorXd>& qd_des,
    const Eigen::Ref<const Eigen::VectorXd>& qdd_des,
    control_msgs::msg::JointTrajectoryControllerState* out) noexcept {
  if (!out) return false;

  const auto n = static_cast<std::size_t>(tau.size());
  if (static_cast<std::size_t>(q.size()) != n) return false;
  if (static_cast<std::size_t>(qd.size()) != n) return false;
  if (static_cast<std::size_t>(q_des.size()) != n) return false;
  if (static_cast<std::size_t>(qd_des.size()) != n) return false;
  if (static_cast<std::size_t>(qdd_des.size()) != n) return false;

  if (!out->joint_names.empty() && out->joint_names.size() != n) return false;

  if (out->reference.positions.size() != n) return false;
  if (out->reference.velocities.size() != n) return false;
  if (out->reference.accelerations.size() != n) return false;

  if (out->feedback.positions.size() != n) return false;
  if (out->feedback.velocities.size() != n) return false;
  if (out->feedback.accelerations.size() != n) return false;

  if (out->error.positions.size() != n) return false;
  if (out->error.velocities.size() != n) return false;

  if (out->output.effort.size() != n) return false;

  out->header.stamp = ros_utils_cpp::time::ToRosTime(stamp_sec);

  for (std::size_t i = 0; i < n; ++i) {
    const int ii = static_cast<int>(i);

    out->reference.positions[i] = q_des[ii];
    out->reference.velocities[i] = qd_des[ii];
    out->reference.accelerations[i] = qdd_des[ii];

    out->feedback.positions[i] = q[ii];
    out->feedback.velocities[i] = qd[ii];
    out->feedback.accelerations[i] = 0.0;

    out->error.positions[i] = q_des[ii] - q[ii];
    out->error.velocities[i] = qd_des[ii] - qd[ii];

    out->output.effort[i] = tau[ii];
  }

  return true;
}

}  // namespace ros_utils_cpp::msg_conv

