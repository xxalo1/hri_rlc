#pragma once

#include "rbt_core_cpp/types.hpp"
#include "rbt_core_cpp/dynamics.hpp"

namespace rbt_core_cpp {

class Controller final {
public:
  using Vec  = rbt_core_cpp::Vec;
  using Mat  = rbt_core_cpp::Mat;
  using Vec3 = rbt_core_cpp::Vec3;
  using Vec6 = rbt_core_cpp::Vec6;
  using Mat6 = rbt_core_cpp::Mat6;
  using Mat4 = rbt_core_cpp::Mat4;
  using Mat6N = rbt_core_cpp::Mat6N;

  explicit Controller(Dynamics& dyn);
  Controller(Dynamics& dyn, pinocchio::FrameIndex tcp_frame_id);

  void set_joint_gains(double Kp, double Kv, double Ki = 0.0);
  void set_joint_gains(const Mat& Kp, const Mat& Kv);
  void set_joint_gains(const Mat& Kp, const Mat& Kv, const Mat& Ki);

  void set_task_gains(double Kx, double Dx, double Kix = 0.0);
  void set_task_gains(const Mat6& Kx, const Mat6& Dx);
  void set_task_gains(const Mat6& Kx, const Mat6& Dx, const Mat6& Kix);

  Vec pid(
    const Eigen::Ref<const Vec>& q,
    const Eigen::Ref<const Vec>& qd,
    const Eigen::Ref<const Vec>& q_des,
    const Eigen::Ref<const Vec>& qd_des,
    double dt
  );

  void pid(
    const Eigen::Ref<const Vec>& q,
    const Eigen::Ref<const Vec>& qd,
    const Eigen::Ref<const Vec>& q_des,
    const Eigen::Ref<const Vec>& qd_des,
    double dt,
    Eigen::Ref<Vec> tau_out
  );

  Vec computed_torque(
    const Eigen::Ref<const Vec>& q,
    const Eigen::Ref<const Vec>& qd,
    const Eigen::Ref<const Vec>& q_des,
    const Eigen::Ref<const Vec>& qd_des,
    const Eigen::Ref<const Vec>& qdd_des
  );

  void computed_torque(
    const Eigen::Ref<const Vec>& q,
    const Eigen::Ref<const Vec>& qd,
    const Eigen::Ref<const Vec>& q_des,
    const Eigen::Ref<const Vec>& qd_des,
    const Eigen::Ref<const Vec>& qdd_des,
    Eigen::Ref<Vec> tau_out
  );

  // xd_des/xdd_des can be nullptr to use zeros.
  Vec impedance_ee(
    const Eigen::Ref<const Vec>& q,
    const Eigen::Ref<const Vec>& qd,
    const Eigen::Ref<const Mat4>& T_des,
    const Vec* xd_des,
    const Vec* xdd_des,
    double dt,
    const Vec* xdd_sec = nullptr,
    const Mat* J_sec = nullptr
  );

  pinocchio::FrameIndex tcp_frame_id() const noexcept { return tcp_frame_id_; }

private:
  static Vec3 orientation_error(
    const Eigen::Matrix3d& R,
    const Eigen::Matrix3d& R_des
  );

  Vec task_to_joint(
    const Eigen::Ref<const Vec>& xdd,
    const Mat6N& J_task,
    const Vec* xdd_sec,
    const Mat* J_sec
  ) const;

private:
  Dynamics& dyn_;
  pinocchio::FrameIndex tcp_frame_id_{0};

  Mat Kp_;
  Mat Kv_;
  Mat Ki_;
  Mat6 Kx_;
  Mat6 Dx_;
  Mat6 Kix_;
  Vec e_int_;
  Vec e_;
  Vec de_;
  Vec v_;
  Vec6 e_task_int_;
};

} // namespace rbt_core_cpp
