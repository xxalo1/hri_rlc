#include "rbt_core_cpp/robot.hpp"

#include <algorithm>

namespace rbt_core_cpp {

Robot::Robot(RobotSpec spec)
    : spec_(std::move(spec)),
      dyn_(Dynamics::FromUrdf(spec_.urdf_path, spec_.tcp_frame)),
      ctrl_(dyn_) {
  const int n = dyn_.n();

  q_model_.setZero(n);
  qd_model_.setZero(n);
  qdd_model_.setZero(n);

  q_des_model_.setZero(n);
  qd_des_model_.setZero(n);
  qdd_des_model_.setZero(n);

  tau_model_.setZero(n);
  tau_src_.setZero(n);

  // Default I/O config: assume source order matches model canonical order.
  // You can override later by calling configure_io().
  configure_io(dyn_.canonical_joint_names());
}

void Robot::set_joint_prefix(const std::string &prefix) {
  joint_prefix_ = prefix;
}

std::string Robot::strip_prefix_if_present(const std::string &name,
                                           const std::string &prefix) {
  if (!prefix.empty() && name.rfind(prefix, 0) == 0)
    return name.substr(prefix.size());
  return name;
}

void Robot::configure_io(const std::vector<std::string> &src_names,
                         const std::string &prefix) {
  if (!prefix.empty())
    joint_prefix_ = prefix;
  else
    joint_prefix_ = "";

  src_joint_names_ = src_names;

  const auto &model_names = dyn_.canonical_joint_names();
  const int n = dyn_.n();

  if (static_cast<int>(src_names.size()) != n) {
    throw std::runtime_error("configure_io: src_names.size() != dyn.n(): got " +
                             std::to_string(src_names.size()) + " expected " +
                             std::to_string(n));
  }

  std::unordered_map<std::string, int> src_index;
  src_index.reserve(src_names.size());

  for (int i = 0; i < static_cast<int>(src_names.size()); ++i) {
    const std::string canon =
        strip_prefix_if_present(src_names[i], joint_prefix_);
    if (src_index.find(canon) != src_index.end()) {
      throw std::runtime_error(
          "configure_io: duplicate joint name after normalization: " + canon);
    }
    src_index.emplace(canon, i);
  }

  // Validate that every model joint is present in src list
  for (const auto &mn : model_names) {
    if (src_index.find(mn) == src_index.end()) {
      throw std::runtime_error("configure_io: missing joint in source: " + mn);
    }
  }

  // src_to_model_idx_[model_i] = src index that corresponds to model joint i
  src_to_model_idx_.assign(n, 0);
  for (int i = 0; i < n; ++i) {
    src_to_model_idx_[i] = src_index.at(model_names[i]);
  }

  // inverse map: model_to_src_idx_[src_i] = model index
  model_to_src_idx_.assign(n, 0);
  for (int model_i = 0; model_i < n; ++model_i) {
    model_to_src_idx_[src_to_model_idx_[model_i]] = model_i;
  }

  io_configured_ = true;
}

void Robot::to_model_inplace(const Eigen::Ref<const Vec> &src,
                             Eigen::Ref<Vec> out_model) const {
  const int n = static_cast<int>(src_to_model_idx_.size());
  for (int i = 0; i < n; ++i) {
    out_model[i] = src[src_to_model_idx_[i]];
  }
}

void Robot::to_src_inplace(const Eigen::Ref<const Vec> &model,
                           Eigen::Ref<Vec> out_src) const {
  const int n = static_cast<int>(model_to_src_idx_.size());
  for (int src_i = 0; src_i < n; ++src_i) {
    out_src[src_i] = model[model_to_src_idx_[src_i]];
  }
}

void Robot::set_joint_state(const Vec *q_src, const Vec *qd_src,
                            const Vec *qdd_src, const double *t) {
  if (q_src) {
    to_model_inplace(*q_src, q_model_);
    dyn_.set_q(q_model_);
  }

  if (qd_src) {
    to_model_inplace(*qd_src, qd_model_);
    dyn_.set_qd(qd_model_);
  }

  if (qdd_src) {
    to_model_inplace(*qdd_src, qdd_model_);
    dyn_.set_qdd(qdd_model_);
  }

  if (t) {
    t_prev_ = t_;
    t_ = *t;
  }
}

void Robot::set_joint_des(const Vec *q_des_src, const Vec *qd_des_src,
                          const Vec *qdd_des_src) {
  if (q_des_src) to_model_inplace(*q_des_src, q_des_model_);
  if (qd_des_src) to_model_inplace(*qd_des_src, qd_des_model_);
  if (qdd_des_src) to_model_inplace(*qdd_des_src, qdd_des_model_);
}

void Robot::set_base_pose_wb(const Eigen::Ref<const Vec> &pose_wb) {
  if (pose_wb.size() != 7)
    throw std::runtime_error(
        "set_base_pose_wb: pose must be size 7 [x y z qw qx qy qz]");

  pose_wb_ = pose_wb;

  const Eigen::Quaterniond q_wb(pose_wb_[3], pose_wb_[4], pose_wb_[5],
                                pose_wb_[6]);  // (w,x,y,z)
  const Eigen::Matrix3d R_wb = q_wb.normalized().toRotationMatrix();

  const Vec3 g_b = R_wb.transpose() * g_w_;
  dyn_.set_gravity(g_b);
}

void Robot::clear_traj() {
  ti_ = 0.0;
  tf_ = 0.0;
  traj_model_ = JointTrajectory{};
  traj_i1_ = 1;
  trace_mode_ = TraceMode::HOLD;
}

bool Robot::has_traj() const noexcept {
  return (t_ >= ti_) && (t_ <= tf_) && (!traj_model_.empty());
}

void Robot::set_joint_traj(const JointTrajectory &traj_src, double duration,
                           double ti) {
  const int n = dyn_.n();
  const int N = traj_src.length();

  if (N <= 0) throw std::runtime_error("set_joint_traj: empty trajectory");

  if (traj_src.q.cols() != n || traj_src.qd.cols() != n ||
      traj_src.qdd.cols() != n) {
    throw std::runtime_error(
        "set_joint_traj: q/qd/qdd must be (N, n_src) with n_src == dyn.n()");
  }
  if (traj_src.q.rows() != N || traj_src.qd.rows() != N ||
      traj_src.qdd.rows() != N) {
    throw std::runtime_error(
        "set_joint_traj: q/qd/qdd row count mismatch with t");
  }

  if (std::isnan(ti)) ti = t_;

  ti_ = ti;
  tf_ = ti + duration;

  traj_model_.t = traj_src.t;
  traj_model_.q.resize(N, n);
  traj_model_.qd.resize(N, n);
  traj_model_.qdd.resize(N, n);

  for (int r = 0; r < N; ++r) {
    for (int i = 0; i < n; ++i) {
      const int src_idx = src_to_model_idx_[i];
      traj_model_.q(r, i) = traj_src.q(r, src_idx);
      traj_model_.qd(r, i) = traj_src.qd(r, src_idx);
      traj_model_.qdd(r, i) = traj_src.qdd(r, src_idx);
    }
  }

  traj_i1_ = 1;
  trace_mode_ = TraceMode::TRAJ;
}

void Robot::sample_joint_traj_into_desired_() {
  const int N = traj_model_.length();
  const int n = dyn_.n();

  if (N <= 0) return;

  const auto &t_arr = traj_model_.t;

  // Clamp outside range
  if (t_ <= t_arr[0]) {
    for (int i = 0; i < n; ++i) {
      q_des_model_[i] = traj_model_.q(0, i);
      qd_des_model_[i] = traj_model_.qd(0, i);
      qdd_des_model_[i] = traj_model_.qdd(0, i);
    }
    return;
  }

  if (t_ >= t_arr[N - 1]) {
    for (int i = 0; i < n; ++i) {
      q_des_model_[i] = traj_model_.q(N - 1, i);
      qd_des_model_[i] = traj_model_.qd(N - 1, i);
      qdd_des_model_[i] = traj_model_.qdd(N - 1, i);
    }
    return;
  }

  // Maintain monotonic index for common 1 kHz stepping.
  if (traj_i1_ < 1) traj_i1_ = 1;

  while (traj_i1_ < N && t_arr[traj_i1_] <= t_) ++traj_i1_;

  const int i1 = traj_i1_;
  const int i0 = i1 - 1;

  const double t0 = t_arr[i0];
  const double t1 = t_arr[i1];
  const double dt = t1 - t0;

  const double alpha = (dt <= 0.0) ? 0.0 : (t_ - t0) / dt;
  const double a0 = 1.0 - alpha;
  const double a1 = alpha;

  for (int i = 0; i < n; ++i) {
    q_des_model_[i] = a0 * traj_model_.q(i0, i) + a1 * traj_model_.q(i1, i);
    qd_des_model_[i] = a0 * traj_model_.qd(i0, i) + a1 * traj_model_.qd(i1, i);
    qdd_des_model_[i] =
        a0 * traj_model_.qdd(i0, i) + a1 * traj_model_.qdd(i1, i);
  }
}

void Robot::update_joint_des_from_traj() {
  if (trace_mode_ == TraceMode::HOLD) return;

  if (has_traj()) {
    sample_joint_traj_into_desired_();
  } else {
    qd_des_model_.setZero();
    qdd_des_model_.setZero();
    trace_mode_ = TraceMode::HOLD;
    clear_traj();
  }
}

const Robot::Vec &Robot::compute_ctrl_effort() {
  // Update desired from trajectory if active.
  if (trace_mode_ == TraceMode::TRAJ) update_joint_des_from_traj();

  switch (ctrl_mode_) {
    case CtrlMode::CT:
      ctrl_.computed_torque(q_model_, qd_model_, q_des_model_, qd_des_model_,
                            qdd_des_model_, tau_model_);
      break;

    case CtrlMode::PID: {
      const double dt = t_ - t_prev_;
      ctrl_.pid(q_model_, qd_model_, q_des_model_, qd_des_model_, dt,
                tau_model_);
      break;
    }

    case CtrlMode::GC:
      tau_model_ = dyn_.g();
      break;

    default:
      tau_model_.setZero();
      break;
  }

  to_src_inplace(tau_model_, tau_src_);
  return tau_src_;
}

}  // namespace rbt_core_cpp
