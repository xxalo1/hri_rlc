#include "rbt_core_cpp/robot.hpp"

#include <algorithm>

namespace rbt_core_cpp {

Robot::Robot(RobotSpec spec)
    : spec_(std::move(spec)),
      dyn_(Dynamics::FromUrdf(spec_.urdf_path, spec_.tcp_frame)),
      ctrl_(dyn_) {
  const int n = dyn_.n();

  state_.resize(n);
  state_des_.resize(n);

  tau_.setZero(n);
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
  if (!prefix.empty()) joint_prefix_ = prefix;

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

void Robot::set_q(const Vec q_src) {
  to_model_inplace(q_src, state_.q);
  dyn_.set_q(state_.q);
}

void Robot::set_qd(const Vec qd_src) {
  to_model_inplace(qd_src, state_.qd);
  dyn_.set_qd(state_.qd);
}

void Robot::set_qdd(const Vec qdd_src) {
  to_model_inplace(qdd_src, state_.qdd);
  dyn_.set_qdd(state_.qdd);
}

void Robot::set_joint_state(const Vec *q_src, const Vec *qd_src,
                            const Vec *qdd_src, const double *t) {
  if (q_src) set_q(*q_src);

  if (qd_src) set_qd(*qd_src);

  if (qdd_src) set_qdd(*qdd_src);

  if (t) {
    t_prev_ = t_;
    t_ = *t;
  }
}

void Robot::set_joint_des(const Vec *q_des_src, const Vec *qd_des_src,
                          const Vec *qdd_des_src) {
  if (q_des_src) to_model_inplace(*q_des_src, state_des_.q);
  if (qd_des_src) to_model_inplace(*qd_des_src, state_des_.qd);
  if (qdd_des_src) to_model_inplace(*qdd_des_src, state_des_.qdd);
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
  traj_ = JointTrajectory{};
  traj_i1_ = 1;
  trace_mode_ = TraceMode::HOLD;
}

bool Robot::has_traj() const noexcept {
  if (traj_.empty()) return false;

  const double t_rel = t_ - ti_;
  const int N = traj_.length();
  if (N <= 0) return false;

  // Keep the trajectory "active" during the delay window (t_rel < t[0]) so the
  // controller holds the first sample until the start time.
  return (t_rel <= traj_.t[N - 1]);
}

void Robot::set_joint_traj(const JointTrajectory &traj_src, double delay) {
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

  if (std::isnan(delay)) delay = 0.0;
  ti_ = t_ + delay;

  // Copy time and validate monotonicity once (NOT in the 1 kHz loop)
  traj_.t = traj_src.t;
  for (int k = 1; k < N; ++k) {
    if (!(traj_.t[k] > traj_.t[k - 1])) {
      throw std::runtime_error("set_joint_traj: t must be strictly increasing");
    }
  }

  tf_ = ti_ + traj_.t[N - 1];

  traj_.q.resize(N, n);
  traj_.qd.resize(N, n);
  traj_.qdd.resize(N, n);

  for (int r = 0; r < N; ++r) {
    for (int i = 0; i < n; ++i) {
      const int src_idx = src_to_model_idx_[i];
      traj_.q(r, i) = traj_src.q(r, src_idx);
      traj_.qd(r, i) = traj_src.qd(r, src_idx);
      traj_.qdd(r, i) = traj_src.qdd(r, src_idx);
    }
  }

  traj_i1_ = 1;
  trace_mode_ = TraceMode::TRAJ;
}

void Robot::sample_joint_traj_into_desired_() {
  const int N = traj_.length();
  const int n = dyn_.n();
  if (N <= 0) return;

  const auto &t_arr = traj_.t;

  // Python semantics: relative time
  const double t_rel = t_ - ti_;

  // Degenerate N==1
  if (N == 1) {
    for (int i = 0; i < n; ++i) {
      state_des_.q[i] = traj_.q(0, i);
      state_des_.qd[i] = traj_.qd(0, i);
      state_des_.qdd[i] = traj_.qdd(0, i);
    }
    return;
  }

  // Clamp (same as Python)
  if (t_rel <= t_arr[0]) {
    for (int i = 0; i < n; ++i) {
      state_des_.q[i] = traj_.q(0, i);
      state_des_.qd[i] = traj_.qd(0, i);
      state_des_.qdd[i] = traj_.qdd(0, i);
    }
    return;
  }

  if (t_rel >= t_arr[N - 1]) {
    for (int i = 0; i < n; ++i) {
      state_des_.q[i] = traj_.q(N - 1, i);
      state_des_.qd[i] = traj_.qd(N - 1, i);
      state_des_.qdd[i] = traj_.qdd(N - 1, i);
    }
    return;
  }

  if (traj_i1_ < 1)
    traj_i1_ = 1;
  else if (traj_i1_ > N - 1)
    traj_i1_ = N - 1;

  while (traj_i1_ < N - 1 && t_arr[traj_i1_] <= t_rel) {
    ++traj_i1_;
  }

  const int i1 = traj_i1_;
  const int i0 = i1 - 1;

  const double t0 = t_arr[i0];
  const double t1 = t_arr[i1];
  const double dt = t1 - t0;

  double alpha = 0.0;
  if (dt > 0.0) {
    alpha = (t_rel - t0) / dt;
  }

  const double a0 = 1.0 - alpha;
  const double a1 = alpha;

  for (int i = 0; i < n; ++i) {
    state_des_.q[i] = a0 * traj_.q(i0, i) + a1 * traj_.q(i1, i);
    state_des_.qd[i] = a0 * traj_.qd(i0, i) + a1 * traj_.qd(i1, i);
    state_des_.qdd[i] = a0 * traj_.qdd(i0, i) + a1 * traj_.qdd(i1, i);
  }
}

void Robot::update_joint_des_from_traj() {
  if (trace_mode_ == TraceMode::HOLD) return;

  if (has_traj()) {
    sample_joint_traj_into_desired_();
  } else {
    state_des_.qd.setZero();
    state_des_.qdd.setZero();
    trace_mode_ = TraceMode::HOLD;
    clear_traj();
  }
}

const Robot::Vec &Robot::compute_ctrl_effort() {
  // Update desired from trajectory if active.
  if (trace_mode_ == TraceMode::TRAJ) update_joint_des_from_traj();

  switch (ctrl_mode_) {
    case CtrlMode::CT:
      ctrl_.computed_torque(state_.q, state_.qd, state_des_.q, state_des_.qd,
                            state_des_.qdd, tau_);
      break;

    case CtrlMode::PID: {
      const double dt = t_ - t_prev_;
      ctrl_.pid(state_.q, state_.qd, state_des_.q, state_des_.qd, dt, tau_);
      break;
    }

    case CtrlMode::GC:
      tau_ = dyn_.g();
      break;

    default:
      tau_.setZero();
      break;
  }

  to_src_inplace(tau_, tau_src_);
  return tau_src_;
}

}  // namespace rbt_core_cpp
