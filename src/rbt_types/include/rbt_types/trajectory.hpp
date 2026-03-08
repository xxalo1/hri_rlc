#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>

#include "rbt_types/math.hpp"

namespace rbt_types
{

/**
 * @brief Start and goal joint states.
 *
 * @details
 * - `start`: Start joint positions [rad], size = dof.
 * - `goal`: Goal joint positions [rad], size = dof.
 */
struct JointEndpoints
{
  JointVec start;
  JointVec goal;
};

/// @brief A collection of joint-space start/goal pairs.
using JointEndpointsSet = std::vector<JointEndpoints>;

/**
 * @brief Joint-space trajectory sampled at a fixed timestep.
 *
 * @details
 * - `states`: Joint positions [rad], shape = (num_waypoints, dof), RowMajor.
 *   Each row is one waypoint in a consistent joint ordering.
 * - `dt`: Sample period [s] between consecutive waypoints.
 */
struct Trajectory
{
  using TrajMat = TrajectoryMat;

  TrajMat states;
  double dt{ 1.0 };

  /// @brief Returns the number of waypoints in `states`.
  std::size_t length() const
  {
    return static_cast<std::size_t>(states.rows());
  }

  /// @brief Returns the joint dimension of each waypoint in `states`.
  std::size_t dim() const
  {
    return static_cast<std::size_t>(states.cols());
  }

  /**
   * @brief Validates trajectory shape and timestep.
   * @throws std::invalid_argument If `states` is empty or `dt <= 0`.
   */
  void validate() const
  {
    if (states.rows() <= 0 || states.cols() <= 0)
    {
      throw std::invalid_argument("Trajectory.states must be non-empty (T>0, d>0).");
    }
    if (!(dt > 0.0))
    {
      throw std::invalid_argument("Trajectory.dt must be > 0.");
    }
  }
};

using TrajectorySet = std::vector<Trajectory>;

}  // namespace rbt_types
