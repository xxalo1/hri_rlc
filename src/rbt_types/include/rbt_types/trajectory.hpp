#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace rbt_types
{

struct Trajectory
{
  using TrajMat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  
  TrajMat states;
  double dt{ 1.0 };

  std::size_t length() const
  {
    return static_cast<std::size_t>(states.rows());
  }
  std::size_t dim() const
  {
    return static_cast<std::size_t>(states.cols());
  }

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