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
#include <cmath>

namespace rbt_core_cpp {

class Dynamics final {
public:
  using Vec  = Eigen::VectorXd;
  using Mat  = Eigen::MatrixXd;
  using Mat6 = Eigen::Matrix<double, 6, Eigen::Dynamic>;
  using Mat4 = Eigen::Matrix4d;

  explicit Dynamics(
    pinocchio::Model model,
    std::string tcp_frame = ""
  );

  static Dynamics FromUrdf(
    const std::string& urdf_path,
    const std::string& tcp_frame = ""
  );

  // Sizes
  int nq() const noexcept { return model_.nq; }
  int nv() const noexcept { return model_.nv; }
  int n()  const noexcept { return model_.nv; }

  const std::vector<std::string>& canonical_joint_names() const noexcept {
    return canonical_joint_names_;
  }

  pinocchio::FrameIndex tcp_frame_id() const noexcept { return tcp_frame_id_; }

  void set_q(const Eigen::Ref<const Vec>& q);

  void set_qd(const Eigen::Ref<const Vec>& qd);
  void set_qdd(const Eigen::Ref<const Vec>& qdd);

  void step(const Vec* q_dof, const Vec* qd, const Vec* qdd);

  void set_gravity(const Eigen::Vector3d& g);

  void compute_fk();
  void compute_joint_jacobians();
  void compute_crba();
  void compute_gravity();

  const Vec& q()   const noexcept { return q_; }
  const Vec& qd()  const noexcept { return qd_; }
  const Vec& qdd() const noexcept { return qdd_; }

  const Mat& M();
  const Vec& g();

  pinocchio::FrameIndex frame_id(const std::string& name) const;

  const pinocchio::SE3& frame_se3(pinocchio::FrameIndex fid);
  Mat4 frame_T(pinocchio::FrameIndex fid);

  const Mat6& frame_jacobian(pinocchio::FrameIndex fid,
                             pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::WORLD);

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

  static void check_size(
    const Eigen::Ref<const Vec>& v, 
    int expected, 
    const char* name
  );

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
