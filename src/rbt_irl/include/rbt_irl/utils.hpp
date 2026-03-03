#pragma once

#include <vector>
#include <Eigen/Core>
#include <cstddef>

#include "rbt_types/trajectory.hpp"

namespace rbt_irl
{
namespace utils
{
using Trajectory = rbt_types::Trajectory;
using TrajectorySet = rbt_types::TrajectorySet;

struct TrajectoryEndpoints
{
  Eigen::RowVectorXd start;
  Eigen::RowVectorXd goal;
};

using TrajectoryEndpointsSet = std::vector<TrajectoryEndpoints>;

inline TrajectoryEndpointsSet extractTrajectoryEndpoints(const TrajectorySet& trajs)
{
  const std::size_t n = trajs.size();

  TrajectoryEndpointsSet out;
  out.reserve(n);

  for (std::size_t i = 0; i < n; ++i)
  {
    const Trajectory& traj = trajs[i];
    out.emplace_back();
    out.back().start = traj.states.row(0);
    out.back().goal = traj.states.row(traj.states.rows() - 1);
  }

  return out;
}

}  // namespace utils
}  // namespace rbt_irl