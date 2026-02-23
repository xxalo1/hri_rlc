#pragma once

#include <optional>
#include <string>

#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "rlc_executive/core/types.hpp"

namespace rlc_executive
{

struct ExecutionFeedback
{
  std::string text;

  double elapsed_sec = 0.0;
};

class TrajectoryExecutor
{
public:
  virtual ~TrajectoryExecutor() = default;

  virtual bool start(const moveit_msgs::msg::RobotTrajectory& traj,
                     const PlanningProfile& profile,
                     std::string* error_msg = nullptr) = 0;

  virtual bool preemptAndStart(const moveit_msgs::msg::RobotTrajectory& traj,
                               const PlanningProfile& profile,
                               std::string* error_msg = nullptr) = 0;

  virtual void requestCancel() = 0;

  virtual bool isActive() const = 0;

  virtual bool hasResult() const = 0;

  virtual std::optional<ExecResult> peekResult() const = 0;

  virtual std::optional<ExecResult> takeResult() = 0;

  virtual ExecutionFeedback latestFeedback() const = 0;
};

}  // namespace rlc_executive