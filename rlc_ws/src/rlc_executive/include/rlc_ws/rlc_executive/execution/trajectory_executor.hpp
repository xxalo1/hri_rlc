#pragma once

#include <string>

#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "rlc_executive/core/types.hpp"

namespace rlc_executive
{

class TrajectoryExecutor
{
public:
  virtual ~TrajectoryExecutor() = default;

  virtual ExecResult execute(const moveit_msgs::msg::RobotTrajectory& traj,
                             const PlanningProfile& profile) = 0;

  virtual void cancel() = 0;
};

}  // namespace rlc_executive