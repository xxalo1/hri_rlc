#include "rbt_planning/trajopt_planner.hpp"

#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rbt_planning {

TrajoptPlanner::TrajoptPlanner(
    std::shared_ptr<tesseract_environment::Environment> env)
    : env_(std::move(env)) {}

void TrajoptPlanner::set_environment(
    std::shared_ptr<tesseract_environment::Environment> env) {
  env_ = std::move(env);
}

void TrajoptPlanner::set_manipulator_group(std::string group) {
  manipulator_group_ = std::move(group);
}

void TrajoptPlanner::set_num_steps(int n_steps) {
  if (n_steps < 2)
    throw std::invalid_argument("set_num_steps: n_steps must be >= 2");
  n_steps_ = n_steps;
}

void TrajoptPlanner::set_start_state(
    const std::vector<std::string>& joint_names,
    const Eigen::VectorXd& q_start) {
  if (!env_) throw std::runtime_error("set_start_state: env_ is null");

  if (static_cast<int>(joint_names.size()) != q_start.size())
    throw std::invalid_argument(
        "set_start_state: joint_names.size() must match q_start.size()");

  joint_names_ = joint_names;
  q_start_ = q_start;

  env_->setState(joint_names_, q_start_);
}

void TrajoptPlanner::set_goal_state(const Eigen::VectorXd& q_goal) {
  if (q_start_.size() == 0)
    throw std::runtime_error("set_goal_state: call set_start_state first");

  if (q_goal.size() != q_start_.size())
    throw std::invalid_argument(
        "set_goal_state: q_goal must match q_start dimension");

  q_goal_ = q_goal;
}

void TrajoptPlanner::validate_traj(const trajopt::TrajArray& seed) const {
  if (seed.rows() != n_steps_)
    throw std::invalid_argument(
        "validate_traj: seed number of rows must match n_steps");

  if (seed.cols() != q_start_.size())
    throw std::invalid_argument(
        "validate_traj: seed number of columns must match ndof");
}

void TrajoptPlanner::set_traj_seed(trajopt::TrajArray&& traj) {
  validate_traj(traj);
  traj_seed_ = std::move(traj);
}

void TrajoptPlanner::add_feature_cost(feature_cost c) {
  if (!c.phi) throw std::invalid_argument("add_feature_cost: phi is empty");
  feature_costs_.push_back(std::move(c));
}

trajopt::TrajArray TrajoptPlanner::solve() {
  if (!env_) throw std::runtime_error("solve: env_ is null");
  if (q_start_.size() == 0 || q_goal_.size() == 0)
    throw std::runtime_error("solve: start/goal not set");

  construct_problem();

  sco::BasicTrustRegionSQP opt(prob_);
  const sco::DblVec x0 = trajopt::trajToDblVec(prob_->GetInitTraj());

  opt.initialize(x0);
  last_status_ = opt.optimize();

  return trajopt::getTraj(opt.x(), prob_->GetVars());
}

void TrajoptPlanner::construct_problem() {
  const int ndof = static_cast<int>(q_start_.size());

  trajopt::ProblemConstructionInfo pci(env_);

  pci.basic_info.n_steps = n_steps_;
  pci.basic_info.manip = manipulator_group_;
  pci.basic_info.start_fixed = true;
  pci.basic_info.use_time = false;

  pci.init_info.type = trajopt::InitInfo::GIVEN_TRAJ;
  pci.init_info.data = traj_seed_;

  auto vel = std::make_shared<trajopt::JointVelCostInfo>();
  vel->term_type = trajopt::TT_COST;
  vel->name = "joint_vel_smooth";
  vel->coeffs = trajopt::DblVec(ndof, 1.0);
  pci.cost_infos.push_back(vel);

  auto goal = std::make_shared<trajopt::JointConstraintInfo>();
  goal->term_type = trajopt::TT_CNT;
  goal->name = "goal_joint_target";
  goal->timestep = n_steps_ - 1;
  goal->coeffs = trajopt::DblVec(ndof, 1.0);
  goal->vals = trajopt::DblVec(q_goal_.data(), q_goal_.data() + q_goal_.size());
  pci.cnt_infos.push_back(goal);

  prob_ = trajopt::ConstructProblem(pci);
  attach_feature_cost(prob_);
}

sco::VarVector TrajoptPlanner::get_vars(
    const boost::shared_ptr<trajopt::TrajOptProb>& prob, const int T,
    const int ndof) const {
  sco::VarVector vars;
  vars.reserve(static_cast<std::size_t>(T * ndof));
  for (int t = 0; t < T; ++t) {
    auto row = prob->GetVarRow(t);
    vars.insert(vars.end(), row.begin(), row.end());
  }
  return vars;
}

void TrajoptPlanner::attach_feature_costs(trajopt::TrajOptProb& prob) const {
  const int T = prob.GetNumSteps();
  const int dof = prob.GetNumDOF();
  auto vars = get_vars(boost::make_shared<trajopt::TrajOptProb>(prob), T, dof);

  for (const auto& c : feature_costs_) {
    auto f =
        sco::ScalarOfVector::construct([T, dof, w = c.weight, phi = c.phi](
                                           const Eigen::VectorXd& x) -> double {
          Eigen::Map<const trajopt::TrajArray> traj(x.data(), T,
                                                    dof);  // view, no copy
          return w * phi(traj);
        });

    auto cost = boost::make_shared<sco::CostFromFunc>(f, vars, c.name);
    prob.addCost(cost);
  }
}

trajopt::TrajArray TrajoptPlanner::make_linear_seed(const Eigen::VectorXd& q0,
                                                    const Eigen::VectorXd& q1,
                                                    int n_steps) {
  if (n_steps < 2)
    throw std::invalid_argument("make_linear_seed: n_steps must be >= 2");
  if (q0.size() != q1.size())
    throw std::invalid_argument("make_linear_seed: q0 and q1 must match");

  const int ndof = static_cast<int>(q0.size());
  trajopt::TrajArray traj(n_steps, ndof);

  for (int t = 0; t < n_steps; ++t) {
    const double a = static_cast<double>(t) / static_cast<double>(n_steps - 1);
    traj.row(t) = ((1.0 - a) * q0 + a * q1).transpose();
  }
  return traj;
}

}  // namespace rbt_planning
