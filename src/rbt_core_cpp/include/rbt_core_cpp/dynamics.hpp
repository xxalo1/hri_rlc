#pragma once

/**
 * @file dynamics.hpp
 * @brief Pinocchio-based rigid-body dynamics wrapper.
 */

#include "rbt_core_cpp/types.hpp"

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>

#include <Eigen/Cholesky>
#include <Eigen/LU>

#include <string>
#include <vector>
#include <cmath>

namespace rbt_core_cpp {

/**
 * @brief Rigid-body dynamics wrapper around Pinocchio model/data with cached computations.
 *
 * @details
 * Size conventions:
 * - `nq()`: Pinocchio configuration size (`model.nq`) used by inverse dynamics (`rnea()`).
 * - `n() == nv()`: number of velocity DOFs (`model.nv`) used for per-joint vectors and matrices.
 *
 * Joint vectors passed to `set_q()`, `set_qd()`, and `set_qdd()` are in *model canonical order* with
 * `size = n()`. Internally, positions are expanded into the Pinocchio configuration vector `q()`
 * (`size = nq()`), including handling of continuous joints (`nq() != n()`).
 *
 * Cached computations are invalidated by `set_q()` and recomputed lazily on demand.
 *
 * @par Thread safety
 * Not thread-safe.
 */
class Dynamics final {
public:
  using Vec  = rbt_core_cpp::Vec;
  using Mat  = rbt_core_cpp::Mat;
  using Mat6 = rbt_core_cpp::Mat6N;
  using Mat4 = rbt_core_cpp::Mat4;

  /**
   * @brief Constructs a dynamics wrapper from an existing Pinocchio model.
   * @param[in] model Pinocchio model to take ownership of (moved into this instance).
   * @param[in] tcp_frame Optional TCP frame name; if empty, uses the last frame in the model.
   * @warning If `tcp_frame` is non-empty and not present in the model, `tcp_frame_id()` will be
   * invalid and calls using it (e.g., `frame_T()`) may throw.
   */
  explicit Dynamics(
    pinocchio::Model model,
    std::string tcp_frame = ""
  );

  /**
   * @brief Builds a Pinocchio model from a URDF file and constructs `Dynamics`.
   * @param[in] urdf_path Path to a URDF file.
   * @param[in] tcp_frame Optional TCP frame name; if empty, uses the last frame in the model.
   * @return A `Dynamics` instance owning the parsed model.
   * @throws std::exception If URDF parsing fails (thrown by Pinocchio).
   */
  static Dynamics FromUrdf(
    const std::string& urdf_path,
    const std::string& tcp_frame = ""
  );

  // Sizes
  /// @brief Returns the Pinocchio configuration size.
  /// @return Configuration dimension (`model.nq`).
  int nq() const noexcept { return model_.nq; }

  /// @brief Returns the velocity DOF count.
  /// @return Velocity dimension (`model.nv`).
  int nv() const noexcept { return model_.nv; }

  /// @brief Returns the joint DOF count used for per-joint vectors.
  /// @return DOF count, equal to `nv()`.
  int n()  const noexcept { return model_.nv; }

  /// @brief Returns joint names in model canonical order.
  /// @return Canonical joint names, size = `n()`.
  const std::vector<std::string>& canonical_joint_names() const noexcept {
    return canonical_joint_names_;
  }

  /// @brief Returns the configured TCP frame id.
  /// @return Pinocchio frame index for the TCP.
  pinocchio::FrameIndex tcp_frame_id() const noexcept { return tcp_frame_id_; }

  /**
   * @brief Sets joint positions and invalidates cached kinematics/dynamics.
   * @param[in] q Joint positions [rad], size = `n()` (model canonical order).
   * @pre `q.size() == n()`.
   * @details Positions are expanded into the Pinocchio configuration vector `q()` (`size = nq()`),
   * including `cos/sin` handling for continuous joints.
   */
  void set_q(const Eigen::Ref<const Vec>& q);

  /**
   * @brief Sets joint velocities.
   * @param[in] qd Joint velocities [rad/s], size = `n()` (model canonical order).
   * @pre `qd.size() == n()`.
   */
  void set_qd(const Eigen::Ref<const Vec>& qd);

  /**
   * @brief Sets joint accelerations.
   * @param[in] qdd Joint accelerations [rad/s^2], size = `n()` (model canonical order).
   * @pre `qdd.size() == n()`.
   */
  void set_qdd(const Eigen::Ref<const Vec>& qdd);

  /**
   * @brief Convenience setter that updates any non-null state pointers.
   * @param[in] q_dof Optional joint positions [rad], size = `n()`; may be nullptr (no update).
   * @param[in] qd Optional joint velocities [rad/s], size = `n()`; may be nullptr (no update).
   * @param[in] qdd Optional joint accelerations [rad/s^2], size = `n()`; may be nullptr (no update).
   */
  void step(const Vec* q_dof, const Vec* qd, const Vec* qdd);

  /**
   * @brief Sets gravity used by Pinocchio for generalized gravity computation.
   * @param[in] g Gravity vector [m/s^2] expressed in the model base frame.
   */
  void set_gravity(const Eigen::Vector3d& g);

  /// @brief Computes forward kinematics for the stored configuration `q()`.
  void compute_fk();

  /// @brief Computes joint Jacobians for the stored configuration `q()`.
  void compute_joint_jacobians();

  /// @brief Computes the joint-space mass matrix for the stored configuration `q()`.
  void compute_crba();

  /// @brief Computes generalized gravity torques for the stored configuration `q()`.
  void compute_gravity();

  /**
   * @brief Returns the stored Pinocchio configuration vector.
   * @return Configuration vector, size = `nq()`.
   */
  const Vec& q()   const noexcept { return q_; }

  /**
   * @brief Returns the stored joint velocity vector.
   * @return Joint velocities [rad/s], size = `n()`.
   */
  const Vec& qd()  const noexcept { return qd_; }

  /**
   * @brief Returns the stored joint acceleration vector.
   * @return Joint accelerations [rad/s^2], size = `n()`.
   */
  const Vec& qdd() const noexcept { return qdd_; }

  /**
   * @brief Returns the joint-space mass matrix for the stored configuration.
   * @return Mass matrix, shape = (`n()`, `n()`) (model canonical order).
   * @note The returned reference points to an internal cache that is recomputed on demand after `set_q()`.
   */
  const Mat& M();

  /**
   * @brief Returns generalized gravity torques for the stored configuration.
   * @return Gravity torques [N·m], size = `n()` (model canonical order).
   * @note The returned reference points into Pinocchio `Data` and is recomputed on demand after `set_q()`.
   */
  const Vec& g();

  /**
   * @brief Looks up a Pinocchio frame index by name.
   * @param[in] name Frame name.
   * @return Frame index. If the name is unknown, Pinocchio returns an invalid index (>= number of frames).
   */
  pinocchio::FrameIndex frame_id(const std::string& name) const;

  /**
   * @brief Returns the world-from-frame pose for a frame.
   * @param[in] fid Frame index.
   * @return World-from-frame transform (`oMf[fid]`).
   * @throws std::runtime_error If `fid` is out of range.
   * @note The returned reference is invalidated by subsequent kinematics updates.
   */
  const pinocchio::SE3& frame_se3(pinocchio::FrameIndex fid);

  /**
   * @brief Returns the world-from-frame homogeneous transform for a frame.
   * @param[in] fid Frame index.
   * @return World-from-frame transform matrix, shape = (4, 4).
   * @throws std::runtime_error If `fid` is out of range.
   */
  Mat4 frame_T(pinocchio::FrameIndex fid);

  /**
   * @brief Returns the 6xN spatial Jacobian for a frame.
   * @param[in] fid Frame index.
   * @param[in] rf Reference frame in which the Jacobian is expressed.
   * @return Frame Jacobian, shape = (6, `n()`), mapping joint velocities to spatial velocity.
   * @note The returned reference points to an internal cache and is overwritten on the next call.
   */
  const Mat6& frame_jacobian(pinocchio::FrameIndex fid,
                             pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::WORLD);

  /**
   * @brief Computes inverse dynamics using Pinocchio RNEA.
   * @param[in] q Pinocchio configuration vector, size = `nq()`.
   * @param[in] qd Joint velocities [rad/s], size = `n()`.
   * @param[in] qdd Joint accelerations [rad/s^2], size = `n()`.
   * @return Generalized forces/torques [N·m], size = `n()`.
   * @note The returned reference points to an internal cache and is overwritten on the next call.
   */
  const Vec& rnea(
    const Eigen::Ref<const Vec>& q,
    const Eigen::Ref<const Vec>& qd,
    const Eigen::Ref<const Vec>& qdd
  );

private:
  void init_joint_packing();
  void invalidate_kinematics();

  void ensure_fk();
  void ensure_jac();

private:
  pinocchio::Model model_;
  pinocchio::Data  data_;

  Vec q_;
  Vec qd_;
  Vec qdd_;

  // Cached compute flags
  bool fk_valid_{false};
  bool jac_valid_{false};
  bool M_valid_{false};
  bool g_valid_{false};

  pinocchio::FrameIndex tcp_frame_id_{0};

  std::vector<int> jids_;
  std::vector<int> idx_q_;
  std::vector<bool> is_cont_;
  std::vector<std::string> canonical_joint_names_;

  Mat  M_cache_;
  Mat6 J_cache_;
  Vec  tau_cache_;

};

} // namespace rbt_core_cpp
