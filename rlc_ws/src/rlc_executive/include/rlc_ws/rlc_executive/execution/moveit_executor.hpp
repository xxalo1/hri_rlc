#pragma once

#include <memory>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "rlc_executive/core/exec_config.hpp"
#include "rlc_executive/core/types.hpp"
#include "rlc_executive/execution/trajectory_executor.hpp"

namespace rlc_executive
{

class MoveItExecutor final : public TrajectoryExecutor
{
public:
  using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
  using Client = rclcpp_action::Client<ExecuteTrajectory>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

  MoveItExecutor(rclcpp::Node& node, const ExecConfig& cfg);

  ExecResult execute(const moveit_msgs::msg::RobotTrajectory& traj,
                     const PlanningProfile& profile) override;

  void cancel() override;

private:
  rclcpp::Node* node_ = nullptr;     // non-owning
  const ExecConfig* cfg_ = nullptr;  // non-owning

  Client::SharedPtr client_;

  // Track the active goal so cancel() can cancel it.
  GoalHandle::SharedPtr active_goal_;
};

}  // namespace rlc_executive