#pragma once

/**
 * @file controller.hpp
 * @brief Joint-space and task-space controllers built on top of `rbt_core_cpp::Dynamics`.
 */

#include "rbt_core_cpp/types.hpp"
#include "rbt_core_cpp/dynamics.hpp"

namespace rbt_core_cpp {

/**
 * @brief Stateful controllers (PID, computed torque, and end-effector impedance) that use a `Dynamics` model.
 *
 * @details
 * All joint-space vectors (`q`, `qd`, `q_des`, ...) are in *model canonical order* with `size = dyn.n()`.
 * Some APIs (e.g., `impedance_ee()`) also require Pinocchio configuration vectors with `size = dyn.nq()`.
 *
 * The controller holds a non-owning reference to `Dynamics` and uses it for gravity, Jacobians, and
 * inverse dynamics. The referenced `Dynamics` instance must outlive the controller.
 *
 * @par Thread safety
 * Not thread-safe.
 */
class Controller final {
public:
  using Vec  = rbt_core_cpp::Vec;
  using Mat  = rbt_core_cpp::Mat;
  using Vec3 = rbt_core_cpp::Vec3;
  using Vec6 = rbt_core_cpp::Vec6;
  using Mat6 = rbt_core_cpp::Mat6;
  using Mat4 = rbt_core_cpp::Mat4;
  using Mat6N = rbt_core_cpp::Mat6N;

  /**
   * @brief Constructs a controller using the `Dynamics` TCP frame.
   * @param[in,out] dyn Dynamics model (non-owning reference); must outlive this instance.
   */
  explicit Controller(Dynamics& dyn);

  /**
   * @brief Constructs a controller using an explicit TCP frame id.
   * @param[in,out] dyn Dynamics model (non-owning reference); must outlive this instance.
   * @param[in] tcp_frame_id TCP frame id in the referenced Pinocchio model.
   */
  Controller(Dynamics& dyn, pinocchio::FrameIndex tcp_frame_id);

  /**
   * @brief Sets joint-space PID gains using isotropic scalars.
   * @param[in] Kp Proportional gain [N·m/rad].
   * @param[in] Kv Derivative gain [N·m/(rad/s)].
   * @param[in] Ki Integral gain [N·m/(rad·s)].
   * @details Applies diagonal gains (`K*I`) and resets the internal joint integrator state.
   */
  void set_joint_gains(double Kp, double Kv, double Ki = 0.0);

  /**
   * @brief Sets joint-space gains as full matrices (no integral term).
   * @param[in] Kp Proportional gain matrix, shape = (dyn.n(), dyn.n()).
   * @param[in] Kv Derivative gain matrix, shape = (dyn.n(), dyn.n()).
   * @pre `Kp.rows() == dyn.n() && Kp.cols() == dyn.n()`.
   * @pre `Kv.rows() == dyn.n() && Kv.cols() == dyn.n()`.
   * @details Resets the internal joint integrator state.
   */
  void set_joint_gains(const Mat& Kp, const Mat& Kv);

  /**
   * @brief Sets joint-space PID gains as full matrices.
   * @param[in] Kp Proportional gain matrix, shape = (dyn.n(), dyn.n()).
   * @param[in] Kv Derivative gain matrix, shape = (dyn.n(), dyn.n()).
   * @param[in] Ki Integral gain matrix, shape = (dyn.n(), dyn.n()).
   * @pre `Kp.rows() == dyn.n() && Kp.cols() == dyn.n()`.
   * @pre `Kv.rows() == dyn.n() && Kv.cols() == dyn.n()`.
   * @pre `Ki.rows() == dyn.n() && Ki.cols() == dyn.n()`.
   * @details Resets the internal joint integrator state.
   */
  void set_joint_gains(const Mat& Kp, const Mat& Kv, const Mat& Ki);

  /**
   * @brief Sets task-space impedance gains using isotropic scalars.
   * @param[in] Kx Stiffness gain on pose error [1/s^2].
   * @param[in] Dx Damping gain on twist error [1/s].
   * @param[in] Kix Integral gain on pose error [1/s^3].
   * @details Applies diagonal gains (`K*I`) and resets the internal task integrator state.
   */
  void set_task_gains(double Kx, double Dx, double Kix = 0.0);

  /**
   * @brief Sets task-space impedance gains as 6x6 matrices (no integral term).
   * @param[in] Kx Stiffness gain matrix, shape = (6, 6).
   * @param[in] Dx Damping gain matrix, shape = (6, 6).
   * @details Resets the internal task integrator state.
   */
  void set_task_gains(const Mat6& Kx, const Mat6& Dx);

  /**
   * @brief Sets task-space impedance gains as 6x6 matrices.
   * @param[in] Kx Stiffness gain matrix, shape = (6, 6).
   * @param[in] Dx Damping gain matrix, shape = (6, 6).
   * @param[in] Kix Integral gain matrix, shape = (6, 6).
   * @details Resets the internal task integrator state.
   */
  void set_task_gains(const Mat6& Kx, const Mat6& Dx, const Mat6& Kix);

  /**
   * @brief Computes joint-space PID + gravity compensation.
   * @param[in] q Joint positions [rad], size = dyn.n() (model canonical order).
   * @param[in] qd Joint velocities [rad/s], size = dyn.n() (model canonical order).
   * @param[in] q_des Desired joint positions [rad], size = dyn.n() (model canonical order).
   * @param[in] qd_des Desired joint velocities [rad/s], size = dyn.n() (model canonical order).
   * @param[in] dt Sample period [s].
   * @return Torque command [N·m], size = dyn.n() (model canonical order).
   * @pre The referenced `Dynamics` state has been updated with `q` via `Dynamics::set_q()`.
   */
  Vec pid(
    const Eigen::Ref<const Vec>& q,
    const Eigen::Ref<const Vec>& qd,
    const Eigen::Ref<const Vec>& q_des,
    const Eigen::Ref<const Vec>& qd_des,
    double dt
  );

  /**
   * @brief Computes joint-space PID + gravity compensation into a caller-provided buffer.
   * @param[in] q Joint positions [rad], size = dyn.n() (model canonical order).
   * @param[in] qd Joint velocities [rad/s], size = dyn.n() (model canonical order).
   * @param[in] q_des Desired joint positions [rad], size = dyn.n() (model canonical order).
   * @param[in] qd_des Desired joint velocities [rad/s], size = dyn.n() (model canonical order).
   * @param[in] dt Sample period [s].
   * @param[out] tau_out Torque command [N·m], size = dyn.n() (model canonical order).
   * @pre `tau_out.size() == dyn.n()`.
   * @pre The referenced `Dynamics` state has been updated with `q` via `Dynamics::set_q()`.
   */
  void pid(
    const Eigen::Ref<const Vec>& q,
    const Eigen::Ref<const Vec>& qd,
    const Eigen::Ref<const Vec>& q_des,
    const Eigen::Ref<const Vec>& qd_des,
    double dt,
    Eigen::Ref<Vec> tau_out
  );

  /**
   * @brief Computes computed-torque (inverse dynamics) control in joint space.
   * @param[in] q Joint positions [rad], size = dyn.n() (model canonical order).
   * @param[in] qd Joint velocities [rad/s], size = dyn.n() (model canonical order).
   * @param[in] q_des Desired joint positions [rad], size = dyn.n() (model canonical order).
   * @param[in] qd_des Desired joint velocities [rad/s], size = dyn.n() (model canonical order).
   * @param[in] qdd_des Desired joint accelerations [rad/s^2], size = dyn.n() (model canonical order).
   * @return Torque command [N·m], size = dyn.n() (model canonical order).
   * @pre The referenced `Dynamics` state has been updated with `q` and `qd` via `Dynamics::set_q()` and `Dynamics::set_qd()`.
   */
  Vec computed_torque(
    const Eigen::Ref<const Vec>& q,
    const Eigen::Ref<const Vec>& qd,
    const Eigen::Ref<const Vec>& q_des,
    const Eigen::Ref<const Vec>& qd_des,
    const Eigen::Ref<const Vec>& qdd_des
  );

  /**
   * @brief Computes computed-torque (inverse dynamics) control into a caller-provided buffer.
   * @param[in] q Joint positions [rad], size = dyn.n() (model canonical order).
   * @param[in] qd Joint velocities [rad/s], size = dyn.n() (model canonical order).
   * @param[in] q_des Desired joint positions [rad], size = dyn.n() (model canonical order).
   * @param[in] qd_des Desired joint velocities [rad/s], size = dyn.n() (model canonical order).
   * @param[in] qdd_des Desired joint accelerations [rad/s^2], size = dyn.n() (model canonical order).
   * @param[out] tau_out Torque command [N·m], size = dyn.n() (model canonical order).
   * @pre `tau_out.size() == dyn.n()`.
   * @pre The referenced `Dynamics` state has been updated with `q` and `qd` via `Dynamics::set_q()` and `Dynamics::set_qd()`.
   */
  void computed_torque(
    const Eigen::Ref<const Vec>& q,
    const Eigen::Ref<const Vec>& qd,
    const Eigen::Ref<const Vec>& q_des,
    const Eigen::Ref<const Vec>& qd_des,
    const Eigen::Ref<const Vec>& qdd_des,
    Eigen::Ref<Vec> tau_out
  );

  /**
   * @brief Computes an end-effector impedance controller torque command.
   * @param[in] q Pinocchio configuration vector, size = dyn.nq().
   * @param[in] qd Joint velocities [rad/s], size = dyn.n() (model canonical order).
   * @param[in] T_des Desired world-from-TCP pose, shape = (4, 4).
   * @param[in] xd_des Optional desired TCP spatial velocity [m/s, rad/s], size = 6; may be nullptr (uses zeros).
   * @param[in] xdd_des Optional desired TCP spatial acceleration [m/s^2, rad/s^2], size = 6; may be nullptr (uses zeros).
   * @param[in] dt Sample period [s].
   * @param[in] xdd_sec Optional secondary-task acceleration, size = m2; may be nullptr (no secondary task).
   * @param[in] J_sec Optional secondary-task Jacobian, shape = (m2, dyn.n()); may be nullptr (no secondary task).
   * @return Torque command [N·m], size = dyn.n() (model canonical order).
   * @pre The referenced `Dynamics` state has been updated with the configuration used to compute the TCP pose/Jacobian.
   * @throws std::runtime_error If matrix sizes are inconsistent or required decompositions fail.
   */
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

  /// @brief Returns the TCP frame id used by `impedance_ee()`.
  /// @return Pinocchio frame index.
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
