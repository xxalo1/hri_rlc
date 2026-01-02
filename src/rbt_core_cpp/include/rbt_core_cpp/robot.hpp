#pragma once

#include "rbt_core_cpp/types.hpp"
#include "rbt_core_cpp/dynamics.hpp"
#include "rbt_core_cpp/controller.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rbt_core_cpp {

/**
 * @brief Control mode (mirrors robot.py CtrlMode).
 */
enum class CtrlMode
{
  CT,   ///< computed torque
  PID,  ///< joint-space PID
  IM,   ///< impedance (placeholder here; Robot does not implement IM yet)
  GC    ///< gravity compensation
};

/**
 * @brief Trajectory trace mode (mirrors robot.py TraceMode).
 */
enum class TraceMode
{
  HOLD,
  TRAJ
};

/**
 * @brief Minimal robot specification (mirrors RobotSpec in robot.py).
 */
struct RobotSpec
{
  std::string name;
  std::string urdf_path;
  std::string tcp_frame;
};

/**
 * @brief Joint trajectory container (time + q/qd/qdd).
 *
 * Notes:
 * - Stored internally in *source* joint order when passed to set_joint_traj(),
 *   then converted once into *model canonical order* for fast sampling at 1 kHz.
 * - Matrices are RowMajor to make row sampling cache-friendly.
 */
struct JointTrajectory
{
  using MatTraj = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  Eigen::VectorXd t;  ///< (N,)
  MatTraj q;          ///< (N, n_src)
  MatTraj qd;         ///< (N, n_src)
  MatTraj qdd;        ///< (N, n_src)

  bool empty() const noexcept { return t.size() == 0; }
  int  length() const noexcept { return static_cast<int>(t.size()); }
};

/**
 * @brief Fast Robot wrapper:
 * - Owns Dynamics and Controller
 * - Maintains I/O joint name mapping (src <-> model canonical order)
 * - Stores current and desired states in model order
 * - Accepts externally generated joint trajectories (no planner inside)
 * - Designed for 1 kHz control loop (no heap allocations in compute_ctrl_effort()).
 *
 * Threading:
 * - Not thread-safe by design (you said you will keep Robot mutated by one thread).
 */
class Robot final
{
public:
  using Vec  = rbt_core_cpp::Vec;
  using Vec3 = rbt_core_cpp::Vec3;

  explicit Robot(RobotSpec spec);

  static Robot FromSpec(const RobotSpec& spec) { return Robot(spec); }

  // This class must not be copied/moved because Controller holds a reference to dyn_.
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;
  Robot(Robot&&) = delete;
  Robot& operator=(Robot&&) = delete;

  // ------------------------
  // Basic info
  // ------------------------
  int n() const noexcept { return dyn_.n(); }
  int nq() const noexcept { return dyn_.nq(); }
  int nv() const noexcept { return dyn_.nv(); }

  const RobotSpec& spec() const noexcept { return spec_; }

  const std::vector<std::string>& canonical_joint_names() const noexcept
  {
    return dyn_.canonical_joint_names();
  }

  const std::vector<std::string>& joint_names_src() const noexcept { return src_joint_names_; }

  // ------------------------
  // I/O mapping (configure once)
  // ------------------------
  void set_joint_prefix(const std::string& prefix);
  void configure_io(const std::vector<std::string>& src_names, const std::string& prefix = "");

  bool io_configured() const noexcept { return io_configured_; }

  // ------------------------
  // State setters (src order in, stored internally in model order)
  // ------------------------
  void set_joint_state(
    const Vec* q_src  = nullptr,
    const Vec* qd_src = nullptr,
    const Vec* qdd_src = nullptr,
    const double* t = nullptr
  );

  void set_joint_des(
    const Vec* q_des_src  = nullptr,
    const Vec* qd_des_src = nullptr,
    const Vec* qdd_des_src = nullptr
  );

  // Base pose / gravity (same conceptual behavior as robot.py set_base_pose)
  void set_base_pose_wb(const Eigen::Ref<const Vec>& pose_wb_xyzwxyz);

  // ------------------------
  // Modes
  // ------------------------
  void set_ctrl_mode(CtrlMode m) noexcept { ctrl_mode_ = m; }
  CtrlMode ctrl_mode() const noexcept { return ctrl_mode_; }

  void set_trace_mode(TraceMode m) noexcept { trace_mode_ = m; }
  TraceMode trace_mode() const noexcept { return trace_mode_; }

  // ------------------------
  // Trajectory (src order in; internally stored in model order)
  // ------------------------
  void clear_traj();
  bool has_traj() const noexcept;

  void set_joint_traj(
    const JointTrajectory& traj_src,
    double duration,
    double ti = std::numeric_limits<double>::quiet_NaN()
  );

  void update_joint_des_from_traj();

  // ------------------------
  // Control
  // ------------------------
  /**
   * @brief Compute control effort in *source joint order* (matches actuator ordering).
   *
   * This function:
   * - Optionally updates desired state from trajectory (if TraceMode::TRAJ).
   * - Runs CT / PID / GC based on CtrlMode.
   * - Returns a reference to an internally cached vector (no allocations).
   */
  const Vec& compute_ctrl_effort();

  const Vec& q_model() const noexcept { return q_model_; }
  const Vec& qd_model() const noexcept { return qd_model_; }
  const Vec& qdd_model() const noexcept { return qdd_model_; }

  const Vec& q_des_model() const noexcept { return q_des_model_; }
  const Vec& qd_des_model() const noexcept { return qd_des_model_; }
  const Vec& qdd_des_model() const noexcept { return qdd_des_model_; }

  double t() const noexcept { return t_; }
  double t_prev() const noexcept { return t_prev_; }

  Dynamics& dyn() noexcept { return dyn_; }
  const Dynamics& dyn() const noexcept { return dyn_; }

  Controller& ctrl() noexcept { return ctrl_; }
  const Controller& ctrl() const noexcept { return ctrl_; }

private:
  // Helpers: src <-> model mapping (no bounds checks in hot path)
  void to_model_inplace(const Eigen::Ref<const Vec>& src, Eigen::Ref<Vec> out_model) const;
  void to_src_inplace(const Eigen::Ref<const Vec>& model, Eigen::Ref<Vec> out_src) const;

  static std::string strip_prefix_if_present(const std::string& name, const std::string& prefix);

  // Trajectory sampling into q_des_model_/qd_des_model_/qdd_des_model_
  void sample_joint_traj_into_desired_();

private:
  RobotSpec spec_;

  Dynamics dyn_;
  Controller ctrl_;

  // World gravity
  Vec3 g_w_{0.0, 0.0, -9.807};

  // Base pose (world_from_base): [x,y,z,qw,qx,qy,qz]
  Vec pose_wb_{Vec::Zero(7)};

  // Time
  double t_{0.0};
  double t_prev_{0.0};

  // I/O mapping
  bool io_configured_{false};
  std::string joint_prefix_;
  std::vector<std::string> src_joint_names_;
  std::vector<int> src_to_model_idx_;  // size n: model_i -> src_index
  std::vector<int> model_to_src_idx_;  // size n: src_i -> model_index

  // Current state (model order, nv-sized)
  Vec q_model_;
  Vec qd_model_;
  Vec qdd_model_;

  // Desired state (model order, nv-sized)
  Vec q_des_model_;
  Vec qd_des_model_;
  Vec qdd_des_model_;

  // Torques (model and source ordering)
  Vec tau_model_;
  Vec tau_src_;

  // Trajectory storage (internally in model order for fast sampling)
  double ti_{0.0};
  double tf_{0.0};
  JointTrajectory traj_model_;
  int traj_i1_{1};  // cached upper index for monotonic time stepping

  // Modes
  CtrlMode ctrl_mode_{CtrlMode::CT};
  TraceMode trace_mode_{TraceMode::HOLD};
};

}
