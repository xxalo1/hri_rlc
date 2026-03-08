#pragma once

#include <tesseract_environment/environment.h>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

#include "rbt_types/math.hpp"
#include "rbt_types/objective_term.hpp"
#include "rbt_types/trajectory.hpp"

namespace rbt_planning
{

class TrajOptPlanner
{
public:
  using JointVec = rbt_types::JointVec;
  using RewardTerms = rbt_types::RewardTerms;
  using Trajectory = rbt_types::Trajectory;
  using TrajectorySet = rbt_types::TrajectorySet;
  using JointEndpointsSet = rbt_types::JointEndpointsSet;
  using JointEndpoints = rbt_types::JointEndpoints;

  struct Options
  {
    double ompl_planning_time{ 5.0 };
    bool ompl_simplify{ false };
    double ompl_longest_valid_segment_fraction{ 0.01 };

    int trajopt_num_steps{ 20 };
    bool use_ompl_seed{ true };

    double dt{ 1.0 };
  };

  explicit TrajOptPlanner(std::shared_ptr<tesseract_environment::Environment> env,
                          const std::string& planning_group);

  explicit TrajOptPlanner(std::shared_ptr<tesseract_environment::Environment> env,
                          const std::string& planning_group, const Options& opt);

  TrajectorySet plan(const JointEndpointsSet& endpoints_set,
                     const RewardTerms& rewards) const;

  /**
   * @brief Plans a joint-space trajectory between start and goal states.
   * @param[in] endpoints Start and goal joint positions [rad], size = dof.
   * @param[in] rewards Reward terms evaluated over the planned trajectory.
   * @return Joint-space trajectory sampled at `Options::dt`.
   * @throws std::runtime_error If planning fails or the environment/group is invalid.
   */
  Trajectory plan(const JointEndpoints& endpoints, const RewardTerms& rewards) const;

private:
  std::shared_ptr<tesseract_environment::Environment> env_;
  std::string planning_group_;
  std::vector<std::string> joint_names_;
  Options opt_{};
};

}  // namespace rbt_planning
