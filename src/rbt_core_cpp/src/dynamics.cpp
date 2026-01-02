#include "rbt_core_cpp/dynamics.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace rbt_core_cpp
{

Dynamics::Dynamics(pinocchio::Model model, std::string tcp_frame)
  : model_(std::move(model)),
    data_(model_),
    q_(Vec::Zero(model_.nq)),
    qd_(Vec::Zero(model_.nv)),
    qdd_(Vec::Zero(model_.nv)),
    M_cache_(Mat::Zero(model_.nv, model_.nv)),
    J_cache_(Mat6::Zero(6, model_.nv)),
    tau_cache_(Vec::Zero(model_.nv))
{
  init_joint_packing();
  invalidate_kinematics();

  if (tcp_frame.empty())
  {
    tcp_frame_id_ = static_cast<pinocchio::FrameIndex>(model_.nframes - 1);
  }
  else
  {
    tcp_frame_id_ = frame_id(tcp_frame);
  }
}

Dynamics Dynamics::FromUrdf(
  const std::string &urdf_path, 
  const std::string &tcp_frame
)
{
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_path, model);
  return Dynamics(std::move(model), tcp_frame);
}

void Dynamics::invalidate_kinematics()
{
  fk_valid_ = false;
  jac_valid_ = false;
  M_valid_ = false;
  g_valid_ = false;
}

void Dynamics::init_joint_packing()
{
  // Equivalent to your python logic:
  // pick joints with nv==1 and nq in {1,2}, sorted by idx_v.
  const auto &joints = model_.joints;

  std::vector<int> tmp;
  tmp.reserve(model_.njoints - 1);

  for (int jid = 1; jid < model_.njoints; ++jid)
  {
    const auto &j = joints[jid];
    if (j.nv() == 1 && (j.nq() == 1 || j.nq() == 2))
    {
      tmp.push_back(jid);
    }
  }

  std::sort(
    tmp.begin(), tmp.end(), [&](int a, int b) { return joints[a].idx_v() < joints[b].idx_v(); });

  jids_ = tmp;
  idx_q_.resize(jids_.size());
  is_cont_.resize(jids_.size());
  canonical_joint_names_.resize(jids_.size());

  for (size_t k = 0; k < jids_.size(); ++k)
  {
    const int jid = jids_[k];
    idx_q_[k] = joints[jid].idx_q();
    is_cont_[k] = (joints[jid].nq() == 2);
    canonical_joint_names_[k] = model_.names[jid];
  }
}

void Dynamics::set_q(const Eigen::Ref<const Vec> &q)
{
  for (int k = 0; k < static_cast<int>(jids_.size()); ++k)
  {
    const int iq = idx_q_[k];
    const double th = q[k];

    if (is_cont_[k])
    {
      q_[iq] = std::cos(th);
      q_[iq + 1] = std::sin(th);
    }
    else
    {
      q_[iq] = th;
    }
  }

  invalidate_kinematics();
}

void Dynamics::set_qd(const Eigen::Ref<const Vec> &qd)
{
  qd_ = qd;
}

void Dynamics::set_qdd(const Eigen::Ref<const Vec> &qdd)
{
  qdd_ = qdd;
}

void Dynamics::step(
  const Vec *q_dof, 
  const Vec *qd, 
  const Vec *qdd
)
{
  if (q_dof)
    set_q(*q_dof);
  if (qd)
    set_qd(*qd);
  if (qdd)
    set_qdd(*qdd);
}

void Dynamics::set_gravity(const Eigen::Vector3d &g)
{
  model_.gravity.linear() = g;
}

void Dynamics::compute_fk()
{
  pinocchio::forwardKinematics(model_, data_, q_);
  pinocchio::updateFramePlacements(model_, data_);
  fk_valid_ = true;
}

void Dynamics::compute_joint_jacobians()
{
  pinocchio::computeJointJacobians(model_, data_, q_);
  jac_valid_ = true;
}

void Dynamics::ensure_fk()
{
  if (!fk_valid_)
    compute_fk();
}

void Dynamics::ensure_jac()
{
  if (!jac_valid_)
    compute_joint_jacobians();
}

void Dynamics::compute_crba()
{
  pinocchio::crba(model_, data_, q_);

  // Pinocchio fills only the upper triangular part for efficiency.
  // Symmetrize into M_cache_ without allocating.
  M_cache_ = data_.M;
  M_cache_.template triangularView<Eigen::StrictlyLower>() =
    M_cache_.transpose().template triangularView<Eigen::StrictlyLower>();

  M_valid_ = true;
}

const Dynamics::Mat &Dynamics::M()
{
  if (!M_valid_)
    compute_crba();
  return M_cache_;
}

void Dynamics::compute_gravity()
{
  pinocchio::computeGeneralizedGravity(model_, data_, q_);
  g_valid_ = true;
}

const Dynamics::Vec &Dynamics::g()
{
  if (!g_valid_)
    compute_gravity();
  return data_.g;
}

pinocchio::FrameIndex Dynamics::frame_id(const std::string &name) const
{
  return model_.getFrameId(name);
}

const pinocchio::SE3 &Dynamics::frame_se3(pinocchio::FrameIndex fid)
{
  ensure_fk();
  if (fid >= static_cast<pinocchio::FrameIndex>(model_.nframes))
    throw std::runtime_error("frame id out of range");
  return data_.oMf[fid];
}

Dynamics::Mat4 Dynamics::frame_T(pinocchio::FrameIndex fid)
{
  const auto &oMf = frame_se3(fid);
  return oMf.toHomogeneousMatrix();
}

const Dynamics::Mat6 &Dynamics::frame_jacobian(
  pinocchio::FrameIndex fid, 
  pinocchio::ReferenceFrame rf
)
{
  ensure_jac();
  pinocchio::getFrameJacobian(
    model_, 
    data_, 
    fid, 
    rf, 
    J_cache_);
  return J_cache_;
}

const Dynamics::Vec &Dynamics::rnea(
  const Eigen::Ref<const Vec> &q, 
  const Eigen::Ref<const Vec> &qd, 
  const Eigen::Ref<const Vec> &qdd
)
{
  tau_cache_ = pinocchio::rnea(model_, data_, q, qd, qdd);
  return tau_cache_;
}

}
