#pragma once

/**
 * @file robot.hpp
 * @brief High-level robot wrapper combining dynamics, control, and joint-order
 * mapping.
 */

#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rbt_core_cpp/controller.hpp"
#include "rbt_core_cpp/dynamics.hpp"
#include "rbt_core_cpp/types.hpp"

namespace rbt_core_cpp {

/**
 * @brief Control mode (mirrors robot.py CtrlMode).
 */
enum class CtrlMode {
  CT,   ///< computed torque
  PID,  ///< joint-space PID
  IM,   ///< impedance (placeholder here; Robot does not implement IM yet)
  GC    ///< gravity compensation
};

/**
 * @brief Trajectory trace mode (mirrors robot.py TraceMode).
 */
enum class TraceMode { HOLD, TRAJ };

/**
 * @brief Minimal robot specification (mirrors RobotSpec in robot.py).
 *
 * @details
 * - `name`: Human-readable robot name (used for logging/identification).
 * - `urdf_path`: Path to the robot URDF used to build the Pinocchio model.
 * - `tcp_frame`: TCP frame name in the model; if empty, the last frame in the
 * - `urdf_source`: Identifies how the URDF is provided; path vs XML string.
 * model is used.
 */
struct RobotSpec {
  std::string name;
  std::string urdf;
  std::string tcp_frame;
  rbt_core_cpp::UrdfSource urdf_source;
};

/**
 * @brief Joint trajectory container (time + q/qd/qdd).
 *
 * @details
 * A trajectory is specified in *source joint order* (I/O/actuator order) and
 * converted once into *model canonical order* when passed to
 * `Robot::set_joint_traj()` for efficient sampling.
 *
 * Sizes/shape:
 * - `t`: time [s], shape = (N,), strictly increasing.
 * - `q`: joint positions [rad], shape = (N, n()) RowMajor (source joint order).
 * - `qd`: joint velocities [rad/s], shape = (N, n()) RowMajor (source joint
 * order).
 * - `qdd`: joint accelerations [rad/s^2], shape = (N, n()) RowMajor (source
 * joint order).
 */
struct JointTrajectory {
  using MatTraj =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  Eigen::VectorXd t;  ///< Time [s], shape = (N,).
  MatTraj q;    ///< Joint positions [rad], shape = (N, n()) RowMajor (source
                ///< joint order).
  MatTraj qd;   ///< Joint velocities [rad/s], shape = (N, n()) RowMajor (source
                ///< joint order).
  MatTraj qdd;  ///< Joint accelerations [rad/s^2], shape = (N, n()) RowMajor
                ///< (source joint order).

  /// @brief Returns true if the trajectory has no samples.
  /// @return True if `t.size() == 0`.
  bool empty() const noexcept { return t.size() == 0; }

  /// @brief Returns the number of trajectory samples.
  /// @return Sample count `N`.
  int length() const noexcept { return static_cast<int>(t.size()); }
};

/**
 * @brief Joint state (model canonical order).
 *
 * @details
 * Sizes: `q`, `qd`, and `qdd` are `size = n()` (model canonical order).
 * - `q`: Joint positions [rad].
 * - `qd`: Joint velocities [rad/s].
 * - `qdd`: Joint accelerations [rad/s^2].
 * - `t`: Sample time [s].
 */
struct JointState {
  Vec q, qd, qdd;
  double t{0.0};

  /**
   * @brief Resizes state vectors and zeros them.
   * @param[in] n Joint DOF count (typically `Robot::n()`).
   */
  void resize(int n) {
    q.setZero(n);
    qd.setZero(n);
    qdd.setZero(n);
  }
};

/**
 * @brief Fast robot wrapper around dynamics + control with source/model
 * joint-order mapping.
 *
 * @details
 * Unless otherwise stated, joint vectors passed into setters are in *source
 * joint order* (I/O/actuator order), `size = n()`. Internally, state is stored
 * in *model canonical order*, `size = n()`.
 *
 * @par Usage
 * @code{.cpp}
 * rbt_core_cpp::RobotSpec spec;
 * spec.name = "GEN3";
 * spec.urdf_path = "/path/to/robot.urdf";
 * spec.tcp_frame = "tool_frame";
 *
 * rbt_core_cpp::Robot robot = rbt_core_cpp::Robot::FromSpec(spec);
 * robot.configure_io(robot.canonical_joint_names());
 *
 * robot.set_joint_state(&q, &qd, nullptr, &t);
 * const rbt_core_cpp::Robot::Vec& tau = robot.compute_ctrl_effort();
 * @endcode
 *
 * @par Thread safety
 * Not thread-safe. External synchronization is required if accessed
 * concurrently.
 *
 * @throws std::runtime_error If I/O mapping or inputs are invalid.
 *
 * @warning
 * Be explicit about joint vector ordering (source vs model) and sizes (`n()`,
 * `nq()`, `nv()`).
 */
class Robot final {
 public:
  using Vec = rbt_core_cpp::Vec;
  using Vec3 = rbt_core_cpp::Vec3;

  /**
   * @brief Constructs a `Robot` from a specification.
   * @param[in] spec Robot specification (URDF + TCP frame).
   * @throws std::exception If URDF parsing fails (thrown by Pinocchio).
   * @throws std::runtime_error If initial I/O mapping fails.
   */
  explicit Robot(RobotSpec spec);

  /**
   * @brief Constructs a `Robot` from a specification.
   * @param[in] spec Robot specification (URDF + TCP frame).
   * @return A `Robot` instance.
   */
  static Robot FromSpec(const RobotSpec& spec) { return Robot(spec); }

  // This class must not be copied/moved because Controller holds a reference to
  // dyn_.
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;
  Robot(Robot&&) = delete;
  Robot& operator=(Robot&&) = delete;

  // ------------------------
  // Basic info
  // ------------------------
  /// @brief Returns the joint DOF count (`nv`).
  /// @return DOF count `n()`.
  int n() const noexcept { return dyn_.n(); }

  /// @brief Returns the Pinocchio configuration size (`nq`).
  /// @return Configuration dimension `nq()`.
  int nq() const noexcept { return dyn_.nq(); }

  /// @brief Returns the Pinocchio velocity size (`nv`).
  /// @return Velocity dimension `nv()`.
  int nv() const noexcept { return dyn_.nv(); }

  /// @brief Returns the robot specification used to construct this instance.
  /// @return Robot specification.
  const RobotSpec& spec() const noexcept { return spec_; }

  /// @brief Returns joint names in model canonical order.
  /// @return Joint names, size = `n()` (model canonical order).
  const std::vector<std::string>& canonical_joint_names() const noexcept {
    return dyn_.canonical_joint_names();
  }

  /// @brief Returns joint names in source (I/O) order.
  /// @return Joint names, size = `n()` (source joint order).
  const std::vector<std::string>& joint_names_src() const noexcept {
    return src_joint_names_;
  }

  // ------------------------
  // I/O mapping (configure once)
  // ------------------------
  /**
   * @brief Sets a joint name prefix used for I/O name normalization.
   * @param[in] prefix Prefix to strip before matching against canonical names.
   * @details This affects how `configure_io()` maps source joint names to model
   * canonical names.
   */
  void set_joint_prefix(const std::string& prefix);

  /**
   * @brief Configures mapping between source joint order and model canonical
   * order.
   * @param[in] src_names Joint names in source (I/O) order, size = `n()`.
   * @param[in] prefix Optional joint name prefix to strip before matching.
   * @throws std::runtime_error If `src_names` is the wrong size, contains
   * duplicates (after prefix stripping), or is missing any canonical joint.
   */
  void configure_io(const std::vector<std::string>& src_names,
                    const std::string& prefix = "");

  /// @brief Returns whether the I/O mapping has been configured.
  /// @return True if I/O mapping is available.
  bool io_configured() const noexcept { return io_configured_; }

  // ------------------------
  // State setters (src order in, stored internally in model order)
  // ------------------------
  /**
   * @brief Updates the robot state from inputs in source joint order.
   * @param[in] q_src Optional joint positions [rad], size = `n()`; may be
   * nullptr (no update).
   * @param[in] qd_src Optional joint velocities [rad/s], size = `n()`; may be
   * nullptr (no update).
   * @param[in] qdd_src Optional joint accelerations [rad/s^2], size = `n()`;
   * may be nullptr (no update).
   * @param[in] t Optional sample time [s]; may be nullptr (no update).
   * @details Updated values are stored internally in model canonical order and
   * forwarded into `dyn()` so that subsequent dynamics/control calls use the
   * new state.
   */
  void set_joint_state(const Vec* q_src = nullptr, const Vec* qd_src = nullptr,
                       const Vec* qdd_src = nullptr, const double* t = nullptr);

  /**
   * @brief Updates the desired joint state from inputs in source joint order.
   * @param[in] q_des_src Optional desired joint positions [rad], size = `n()`;
   * may be nullptr (no update).
   * @param[in] qd_des_src Optional desired joint velocities [rad/s], size =
   * `n()`; may be nullptr (no update).
   * @param[in] qdd_des_src Optional desired joint accelerations [rad/s^2], size
   * = `n()`; may be nullptr (no update).
   * @details Desired values are stored internally in model canonical order.
   */
  void set_joint_des(const Vec* q_des_src = nullptr,
                     const Vec* qd_des_src = nullptr,
                     const Vec* qdd_des_src = nullptr);

  /**
   * @brief Sets joint positions (source order) and updates `dyn()`.
   * @param[in] q_src Joint positions [rad], size = `n()` (source joint order).
   */
  void set_q(const Vec q_src);

  /**
   * @brief Sets joint velocities (source order) and updates `dyn()`.
   * @param[in] qd_src Joint velocities [rad/s], size = `n()` (source joint
   * order).
   */
  void set_qd(const Vec qd_src);

  /**
   * @brief Sets joint accelerations (source order) and updates `dyn()`.
   * @param[in] qdd_src Joint accelerations [rad/s^2], size = `n()` (source
   * joint order).
   */
  void set_qdd(const Vec qdd_src);

  // Base pose / gravity (same conceptual behavior as robot.py set_base_pose)
  /**
   * @brief Sets base pose and updates gravity accordingly.
   * @param[in] pose_wb_xyzwxyz World-from-base pose, size = 7 [x y z qw qx qy
   * qz].
   * @throws std::runtime_error If `pose_wb_xyzwxyz.size() != 7`.
   */
  void set_base_pose_wb(const Eigen::Ref<const Vec>& pose_wb_xyzwxyz);

  // ------------------------
  // Modes
  // ------------------------
  /// @brief Sets the active control mode used by `compute_ctrl_effort()`.
  /// @param[in] m Control mode.
  void set_ctrl_mode(CtrlMode m) noexcept { ctrl_mode_ = m; }

  /// @brief Returns the active control mode.
  /// @return Control mode.
  CtrlMode ctrl_mode() const noexcept { return ctrl_mode_; }

  /// @brief Sets the active trace mode used for desired-state updates.
  /// @param[in] m Trace mode.
  void set_trace_mode(TraceMode m) noexcept { trace_mode_ = m; }

  /// @brief Returns the active trace mode.
  /// @return Trace mode.
  TraceMode trace_mode() const noexcept { return trace_mode_; }

  // ------------------------
  // Trajectory (src order in; internally stored in model order)
  // ------------------------
  /// @brief Clears any active trajectory and switches to hold mode.
  void clear_traj();

  /// @brief Returns true if a trajectory is present and active at the current
  /// time.
  /// @return True if a trajectory is currently active.
  bool has_traj() const noexcept;

  /**
   * @brief Sets an externally generated joint trajectory (inputs in source
   * joint order).
    * @param[in] traj_src Joint trajectory (shared ownership):
   *   - `t`: time [s], shape = (N,), strictly increasing.
   *   - `q`: joint positions [rad], shape = (N, `n()`) RowMajor (source joint
   * order).
   *   - `qd`: joint velocities [rad/s], shape = (N, `n()`) RowMajor (source
   * joint order).
   *   - `qdd`: joint accelerations [rad/s^2], shape = (N, `n()`) RowMajor
   * (source joint order).
   * @param[in] delay Start delay [s].
    * @details
    * This stores a pointer to the provided trajectory. The
    * trajectory must not be modified after being passed in.
    *
    * @throws std::runtime_error If `traj_src` is null, the trajectory is empty,
    * shapes mismatch, or `t` is not strictly increasing.
   */
    void set_joint_traj(std::shared_ptr<const JointTrajectory> traj_src,
                 double delay = 0.0);

  /// @brief Updates the desired state from the active trajectory (if in
  /// `TraceMode::TRAJ`).
  void update_joint_des_from_traj();

  // ------------------------
  // Control
  // ------------------------
  /**
   * @brief Computes control effort in *source joint order* (matches actuator
   * ordering).
   * @return Torque command [N·m], size = `n()` (source joint order).
   * @note The returned reference is valid until the next call to
   * `compute_ctrl_effort()`.
   *
   * @details
   * This function optionally updates desired state from the active trajectory
   * (`TraceMode::TRAJ`) and runs CT / PID / GC based on `CtrlMode`.
   */
  const Vec& compute_ctrl_effort();

  /// @brief Returns the current joint positions in model canonical order.
  /// @return Joint positions [rad], size = `n()` (model canonical order).
  const Vec& q_model() const noexcept { return state_.q; }

  /// @brief Returns the current joint velocities in model canonical order.
  /// @return Joint velocities [rad/s], size = `n()` (model canonical order).
  const Vec& qd_model() const noexcept { return state_.qd; }

  /// @brief Returns the current joint accelerations in model canonical order.
  /// @return Joint accelerations [rad/s^2], size = `n()` (model canonical
  /// order).
  const Vec& qdd_model() const noexcept { return state_.qdd; }

  /// @brief Returns the desired joint positions in model canonical order.
  /// @return Desired joint positions [rad], size = `n()` (model canonical
  /// order).
  const Vec& q_des_model() const noexcept { return state_des_.q; }

  /// @brief Returns the desired joint velocities in model canonical order.
  /// @return Desired joint velocities [rad/s], size = `n()` (model canonical
  /// order).
  const Vec& qd_des_model() const noexcept { return state_des_.qd; }

  /// @brief Returns the desired joint accelerations in model canonical order.
  /// @return Desired joint accelerations [rad/s^2], size = `n()` (model
  /// canonical order).
  const Vec& qdd_des_model() const noexcept { return state_des_.qdd; }

  /// @brief Returns the current sample time.
  /// @return Time [s].
  double t() const noexcept { return t_; }

  /// @brief Returns the previous sample time.
  /// @return Previous time [s].
  double t_prev() const noexcept { return t_prev_; }

  /// @brief Returns a mutable reference to the owned dynamics model.
  /// @return Dynamics instance.
  Dynamics& dyn() noexcept { return dyn_; }

  /// @brief Returns a const reference to the owned dynamics model.
  /// @return Dynamics instance.
  const Dynamics& dyn() const noexcept { return dyn_; }

  /// @brief Returns a mutable reference to the owned controller.
  /// @return Controller instance.
  Controller& ctrl() noexcept { return ctrl_; }

  /// @brief Returns a const reference to the owned controller.
  /// @return Controller instance.
  const Controller& ctrl() const noexcept { return ctrl_; }

 private:
  // Helpers: src <-> model mapping (no bounds checks in hot path)
  void to_model_inplace(const Eigen::Ref<const Vec>& src,
                        Eigen::Ref<Vec> out_model) const;
  void to_src_inplace(const Eigen::Ref<const Vec>& model,
                      Eigen::Ref<Vec> out_src) const;

  static std::string strip_prefix_if_present(const std::string& name,
                                             const std::string& prefix);

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
  JointState state_;

  // Desired state (model order, nv-sized)
  JointState state_des_;

  // Torques (model and source ordering)
  Vec tau_;
  Vec tau_src_;

  // Trajectory storage (internally in model order for fast sampling)
  double ti_{0.0};
  double tf_{0.0};
  std::shared_ptr<const JointTrajectory> traj_;
  int traj_i1_{1};  // cached upper index for monotonic time stepping

  // Modes
  CtrlMode ctrl_mode_{CtrlMode::CT};
  TraceMode trace_mode_{TraceMode::HOLD};
};

}  // namespace rbt_core_cpp
