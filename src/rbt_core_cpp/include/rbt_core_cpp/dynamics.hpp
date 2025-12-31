#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/LU>

#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>

namespace rbt_core_cpp {

class Dynamics final {
public:
  using Vec  = Eigen::VectorXd;
  using Mat  = Eigen::MatrixXd;
  using Mat6 = Eigen::Matrix<double, 6, Eigen::Dynamic>;
  using Mat4 = Eigen::Matrix4d;

  // Construct from an already built model
  explicit Dynamics(pinocchio::Model model,
                             std::string tcp_frame = "");

  // Build from URDF (no geometry needed for dynamics/kinematics)
  static Dynamics FromUrdf(const std::string& urdf_path,
                                    const std::string& tcp_frame = "",
                                    bool floating_base = false);

  // Sizes
  int nq() const noexcept { return model_.nq; }
  int nv() const noexcept { return model_.nv; }
  int n()  const noexcept { return model_.nv; }  // like your python wrapper

  // Canonical 1-DoF joint names (ordered by idx_v)
  const std::vector<std::string>& canonical_joint_names() const noexcept {
    return canonical_joint_names_;
  }

  // Pack 1-DoF joint positions into Pinocchio q buffer (handles nq=2 joints)
  // q_dof has size = n() = nv()
  void set_q_dof(const Eigen::Ref<const Vec>& q_dof);

  // Velocities/accelerations are nv() always
  void set_qd(const Eigen::Ref<const Vec>& qd);
  void set_qdd(const Eigen::Ref<const Vec>& qdd);

  // Convenience step
  void step(const Vec* q_dof, const Vec* qd, const Vec* qdd);

  // Gravity
  void set_gravity(const Eigen::Vector3d& g);

  // Compute primitives (cached)
  void compute_fk();
  void compute_joint_jacobians();
  void compute_crba();               // fills data_.M (upper triangle) and symmetrizes
  void compute_nle();                // fills data_.nle
  void compute_gravity();            // fills data_.g

  // Accessors (make sure you computed what you need)
  const Vec& q()   const noexcept { return q_; }
  const Vec& qd()  const noexcept { return qd_; }
  const Vec& qdd() const noexcept { return qdd_; }

  const Mat& M();    // mass matrix (symmetrized)
  const Vec& nle();  // nonlinear effects
  const Vec& g();    // gravity

  // Frames and Jacobians
  pinocchio::FrameIndex frame_id(const std::string& name) const;

  const pinocchio::SE3& frame_se3(pinocchio::FrameIndex fid);  // world -> frame
  Mat4 frame_T(pinocchio::FrameIndex fid);                     // homogeneous

  // Returns a reference to an internal buffer J_ (6 x nv)
  // reference frame: WORLD by default
  const Mat6& frame_jacobian(pinocchio::FrameIndex fid,
                             pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::WORLD);

  // RNEA (returns tau, no allocations)
  const Vec& rnea(const Eigen::Ref<const Vec>& q,
                  const Eigen::Ref<const Vec>& qd,
                  const Eigen::Ref<const Vec>& qdd);

private:
  void init_joint_packing();
  void invalidate_kinematics();

  void ensure_fk();
  void ensure_jac();

  static void check_size(const Eigen::Ref<const Vec>& v, int expected, const char* name);

private:
  pinocchio::Model model_;
  pinocchio::Data  data_;

  // Pinocchio buffers (allocated once)
  Vec q_;     // size nq
  Vec qd_;    // size nv
  Vec qdd_;   // size nv

  // Cached compute flags
  bool fk_valid_{false};
  bool jac_valid_{false};
  bool M_valid_{false};
  bool nle_valid_{false};
  bool g_valid_{false};

  // TCP
  pinocchio::FrameIndex tcp_frame_id_{0};

  // Joint packing (1-DoF joints)
  std::vector<int> jids_;
  std::vector<int> idx_q_;
  std::vector<bool> is_cont_;
  std::vector<std::string> canonical_joint_names_;

  // Internal work buffers (preallocated)
  Mat  M_cache_;      // nv x nv
  Mat6 J_cache_;      // 6 x nv
  Vec  tau_cache_;    // nv
  Vec  qdd_task_;     // nv

  // For dyn_pinv computations (avoid temp allocations)
  Mat  X_;            // nv x task_dim
  Mat  Lambda_inv_;   // task_dim x task_dim
};

} // namespace rbt_core_cpp
