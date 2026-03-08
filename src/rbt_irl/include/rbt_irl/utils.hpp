#pragma once

#include <cstddef>

#include "rbt_types/trajectory.hpp"

namespace rbt_irl
{
namespace utils
{
using Trajectory = rbt_types::Trajectory;
using TrajectorySet = rbt_types::TrajectorySet;
using JointVec = rbt_types::JointVec;
using TrajectoryEndpoints = rbt_types::JointEndpoints;
using TrajectoryEndpointsSet = rbt_types::JointEndpointsSet;

/**
 * @brief Extracts start/goal joint states from each demonstration trajectory.
 * @param[in] trajs Demonstration trajectories, length = num_demos.
 * @return Extracted start/goal pairs, length = `trajs.size()`.
 */
inline TrajectoryEndpointsSet extractTrajectoryEndpoints(const TrajectorySet& trajs)
{
  const std::size_t n = trajs.size();

  TrajectoryEndpointsSet out;
  out.reserve(n);

  for (std::size_t i = 0; i < n; ++i)
  {
    const Trajectory& traj = trajs[i];
    out.emplace_back();
    out.back().start = traj.states.row(0).transpose();
    out.back().goal = traj.states.row(traj.states.rows() - 1).transpose();
  }

  return out;
}

}  // namespace utils
}  // namespace rbt_irl
