#include "rbt_planning/trajopt_planner.hpp"

#include "rbt_planning/ompl_solver.hpp"
#include "rbt_planning/trajopt_solver.hpp"

#include <stdexcept>
#include <utility>

#include "rbt_planning/utils.hpp"

namespace rbt_planning
{

TrajOptPlanner::TrajOptPlanner(std::shared_ptr<tesseract_environment::Environment> env,
                               const std::string& planning_group)
  : TrajOptPlanner(std::move(env), planning_group, Options{})
{
}

TrajOptPlanner::TrajOptPlanner(std::shared_ptr<tesseract_environment::Environment> env,
                               const std::string& planning_group, const Options& opt)
  : env_(std::move(env)), planning_group_(planning_group), opt_(opt)
{
  if (!env_)
  {
    throw std::invalid_argument("TrajOptPlanner: env is null");
  }

  joint_names_ = env_->getGroupJointNames(planning_group_);
  if (joint_names_.empty())
  {
    throw std::runtime_error("TrajOptPlanner: env returned no joint names for group: " +
                             planning_group_);
  }
}

TrajOptPlanner::TrajectorySet TrajOptPlanner::plan(const JointEndpointsSet& endpoints_set,
                                                   const RewardTerms& rewards) const
{
  TrajectorySet trajs;
  trajs.reserve(endpoints_set.size());
  for (const JointEndpoints& endpoint : endpoints_set)
  {
    Trajectory traj = plan(endpoint, rewards);
    trajs.push_back(std::move(traj));
  }
  return trajs;
}

TrajOptPlanner::Trajectory TrajOptPlanner::plan(const JointEndpoints& endpoints,
                                                const RewardTerms& rewards) const
{
  const JointVec& q_start = endpoints.start;
  const JointVec& q_goal = endpoints.goal;

  const std::size_t dof = static_cast<std::size_t>(q_start.size());

  std::shared_ptr<const trajopt::TrajArray> seed_ptr;
  if (opt_.use_ompl_seed)
  {
    OMPLSolver ompl(env_);
    ompl.setManipulatorGroup(planning_group_);
    ompl.setPlanningTime(opt_.ompl_planning_time);
    ompl.setSimplify(opt_.ompl_simplify);
    ompl.setLongestValidSegmentFraction(opt_.ompl_longest_valid_segment_fraction);

    ompl.setStartState(joint_names_, q_start);
    ompl.setGoalState(q_goal);

    const auto ss = ompl.solve();
    if (ss && ss->getLastPlannerStatus())
    {
      trajopt::TrajArray seed;
      const auto ok =
          utils::fromOMPL(ss->getSolutionPath(), opt_.trajopt_num_steps, dof, seed);
      if (ok)
      {
        seed_ptr = std::make_shared<const trajopt::TrajArray>(std::move(seed));
      }
    }
  }

  TrajOptSolver trajopt(env_);
  trajopt.setManipulatorGroup(planning_group_);
  trajopt.setNumSteps(opt_.trajopt_num_steps);
  trajopt.setStartState(joint_names_, q_start);
  trajopt.setGoalState(q_goal);

  if (seed_ptr)
  {
    trajopt.setTrajSeed(seed_ptr);
  }

  trajopt.addCost(rbt_types::rewardToCost(rewards));

  const trajopt::TrajArray traj = trajopt.solve();

  Trajectory out;
  out.states = traj;
  out.dt = opt_.dt;
  out.validate();
  return out;
}

}  // namespace rbt_planning
