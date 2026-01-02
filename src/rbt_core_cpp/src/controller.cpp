#include "rbt_core_cpp/controller.hpp"

#include <Eigen/Cholesky>

#include <pinocchio/algorithm/frames.hpp>

#include <stdexcept>
#include <string>

namespace rbt_core_cpp
{

Controller::Controller(Dynamics &dyn)
  : Controller(dyn, dyn.tcp_frame_id())
{

}

Controller::Controller(Dynamics &dyn, pinocchio::FrameIndex tcp_frame_id)
  : dyn_(dyn),
    tcp_frame_id_(tcp_frame_id)
{
  set_joint_gains(5.0, 2.0, 0.0);

  set_task_gains(1.0, 2.0, 0.0);

  const int n = dyn_.n();
  e_.setZero(n);
  de_.setZero(n);
  v_.setZero(n);
}

Controller::Vec3 Controller::orientation_error(
  const Eigen::Matrix3d &R, 
  const Eigen::Matrix3d &R_des
)
{
  const Eigen::Matrix3d R_err = R_des.transpose() * R;
  Vec3 e;
  e << (R_err(2, 1) - R_err(1, 2)),
       (R_err(0, 2) - R_err(2, 0)),
       (R_err(1, 0) - R_err(0, 1));
  return 0.5 * e;
}

void Controller::set_joint_gains(double Kp, double Kv, double Ki)
{
  const int n = dyn_.n();
  const Mat I = Mat::Identity(n, n);
  Kp_ = Kp * I;
  Kv_ = Kv * I;
  Ki_ = Ki * I;
  e_int_.setZero(n);
}

void Controller::set_joint_gains(const Mat &Kp, const Mat &Kv)
{
  set_joint_gains(Kp, Kv, Mat::Zero(Kp.rows(), Kp.cols()));
}

void Controller::set_joint_gains(const Mat &Kp, const Mat &Kv, const Mat &Ki)
{
  const int n = dyn_.n();
  Kp_ = Kp;
  Kv_ = Kv;
  Ki_ = Ki;
  e_int_.setZero(n);
}

void Controller::set_task_gains(double Kx, double Dx, double Kix)
{
  Kx_ = Kx * Mat6::Identity();
  Dx_ = Dx * Mat6::Identity();
  Kix_ = Kix * Mat6::Identity();
  e_task_int_.setZero();
}

void Controller::set_task_gains(const Mat6 &Kx, const Mat6 &Dx)
{
  set_task_gains(Kx, Dx, Mat6::Zero());
}

void Controller::set_task_gains(const Mat6 &Kx, const Mat6 &Dx, const Mat6 &Kix)
{
  Kx_ = Kx;
  Dx_ = Dx;
  Kix_ = Kix;
  e_task_int_.setZero();
}

Controller::Vec Controller::pid(
  const Eigen::Ref<const Vec> &q,
  const Eigen::Ref<const Vec> &qd,
  const Eigen::Ref<const Vec> &q_des,
  const Eigen::Ref<const Vec> &qd_des,
  double dt
)
{
  Vec tau(dyn_.n());
  pid(q, qd, q_des, qd_des, dt, tau);
  return tau;
}

void Controller::pid(
  const Eigen::Ref<const Vec> &q,
  const Eigen::Ref<const Vec> &qd,
  const Eigen::Ref<const Vec> &q_des,
  const Eigen::Ref<const Vec> &qd_des,
  double dt,
  Eigen::Ref<Vec> tau_out
)
{
  e_.noalias() = q_des - q;
  de_.noalias() = qd_des - qd;
  e_int_.noalias() += e_ * dt;

  tau_out.noalias() = Kp_ * e_ + Kv_ * de_ + Ki_ * e_int_;
  tau_out += dyn_.g();
}

Controller::Vec Controller::computed_torque(
  const Eigen::Ref<const Vec> &q,
  const Eigen::Ref<const Vec> &qd,
  const Eigen::Ref<const Vec> &q_des,
  const Eigen::Ref<const Vec> &qd_des,
  const Eigen::Ref<const Vec> &qdd_des
)
{
  Vec tau(dyn_.n());
  computed_torque(q, qd, q_des, qd_des, qdd_des, tau);
  return tau;
}

void Controller::computed_torque(
  const Eigen::Ref<const Vec> &q,
  const Eigen::Ref<const Vec> &qd,
  const Eigen::Ref<const Vec> &q_des,
  const Eigen::Ref<const Vec> &qd_des,
  const Eigen::Ref<const Vec> &qdd_des,
  Eigen::Ref<Vec> tau_out
)
{
  e_.noalias() = q_des - q;
  de_.noalias() = qd_des - qd;
  v_.noalias() = qdd_des + Kv_ * de_ + Kp_ * e_;

  tau_out = dyn_.rnea(dyn_.q(), dyn_.qd(), v_);
}

Controller::Vec Controller::task_to_joint(
  const Eigen::Ref<const Vec> &xdd,
  const Mat6N &J_task,
  const Vec *xdd_sec,
  const Mat *J_sec
) const
{
  const int n = dyn_.n();

  if (J_task.cols() != n)
  {
    throw std::runtime_error(
      "J_task size mismatch: got " +
      std::to_string(J_task.rows()) + "x" + std::to_string(J_task.cols()) +
      " expected 6x" + std::to_string(n));
  }

  const Mat &M = dyn_.M();

  Eigen::LDLT<Mat> M_ldlt(M);
  if (M_ldlt.info() != Eigen::Success)
  {
    throw std::runtime_error("M LDLT decomposition failed");
  }
  const Mat X = M_ldlt.solve(J_task.transpose()); // (n x m1)
  const Mat Lambda_inv = J_task * X;              // (m1 x m1)

  Eigen::LDLT<Mat> Lambda_ldlt(Lambda_inv);
  if (Lambda_ldlt.info() != Eigen::Success)
  {
    throw std::runtime_error("Lambda LDLT decomposition failed");
  }
  const Mat Lambda = Lambda_ldlt.solve(Mat::Identity(Lambda_inv.rows(), Lambda_inv.cols()));

  const Mat J1_dyn = X * Lambda;  // (n x m1)
  const Vec qdd_main = J1_dyn * xdd;

  Vec qdd_sec = Vec::Zero(n);
  if (xdd_sec != nullptr && J_sec != nullptr)
  {
    if (J_sec->cols() != n)
    {
      throw std::runtime_error(
        "J_sec size mismatch: got " +
        std::to_string(J_sec->rows()) + "x" + std::to_string(J_sec->cols()) +
        " expected m2x" + std::to_string(n));
    }

    const Mat JJt = (*J_sec) * J_sec->transpose();
    Eigen::LDLT<Mat> JJt_ldlt(JJt);
    if (JJt_ldlt.info() != Eigen::Success)
    {
      throw std::runtime_error("J_sec JJt LDLT decomposition failed");
    }
    const Mat J2_pinv = J_sec->transpose() *
      JJt_ldlt.solve(Mat::Identity(JJt.rows(), JJt.cols()));

    const Vec qdd_sec_candidate = J2_pinv * (*xdd_sec);
    const Mat N1 = Mat::Identity(n, n) - J1_dyn * J_task;
    qdd_sec = N1 * qdd_sec_candidate;
  }

  return qdd_main + qdd_sec;
}

Controller::Vec Controller::impedance_ee(
  const Eigen::Ref<const Vec> &q,
  const Eigen::Ref<const Vec> &qd,
  const Eigen::Ref<const Mat4> &T_des,
  const Vec *xd_des,
  const Vec *xdd_des,
  double dt,
  const Vec *xdd_sec,
  const Mat *J_sec
)
{
  const Mat4 T_tcp = dyn_.frame_T(tcp_frame_id_);

  const Eigen::Matrix3d R = T_tcp.block<3, 3>(0, 0);
  const Eigen::Vector3d p = T_tcp.block<3, 1>(0, 3);

  const Eigen::Matrix3d R_des = T_des.block<3, 3>(0, 0);
  const Eigen::Vector3d p_des = T_des.block<3, 1>(0, 3);

  const Eigen::Vector3d e_pos = p_des - p;
  const Eigen::Vector3d e_ori = orientation_error(R, R_des);

  Vec6 e;
  e << e_pos, e_ori;

  const Mat6N &J_tcp = dyn_.frame_jacobian(
    tcp_frame_id_, 
    pinocchio::ReferenceFrame::WORLD);

  const Vec6 xd = J_tcp * qd;
  Vec6 xd_des_local = Vec6::Zero();
  Vec6 xdd_des_local = Vec6::Zero();
  if (xd_des != nullptr)
    xd_des_local = *xd_des;
  if (xdd_des != nullptr)
    xdd_des_local = *xdd_des;

  const Vec6 de = xd_des_local - xd;
  e_task_int_ += e * dt;

  const Vec6 xdd_main = xdd_des_local + Dx_ * de + Kx_ * e + Kix_ * e_task_int_;

  const Vec qdd_star = task_to_joint(xdd_main, J_tcp, xdd_sec, J_sec);
  const Vec tau = dyn_.rnea(q, qd, qdd_star);

  return tau;
}

}
