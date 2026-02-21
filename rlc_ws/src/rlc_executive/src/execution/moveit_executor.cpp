#include "rlc_executive/execution/moveit_executor.hpp"

#include <chrono>
#include <future>
#include <string>

namespace rlc_executive
{

MoveItExecutor::MoveItExecutor(rclcpp::Node& node, const ExecConfig& cfg)
  : node_(&node), cfg_(&cfg)
{
  client_ = rclcpp_action::create_client<ExecuteTrajectory>(
      node_, cfg_->execute_traj_action_name);
}

ExecResult MoveItExecutor::execute(const moveit_msgs::msg::RobotTrajectory& traj,
                                   const PlanningProfile& /*profile*/)
{
  ExecResult out;

  if (!client_)
  {
    out.status = ExecStatus::FAILURE;
    out.message = "MoveItExecutor: client_ is null";
    return out;
  }

  if (!client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    out.status = ExecStatus::FAILURE;
    out.message = "MoveItExecutor: ExecuteTrajectory action server not available: " +
                  cfg_->execute_traj_action_name;
    return out;
  }

  ExecuteTrajectory::Goal goal;
  goal.trajectory = traj;

  const auto t_start = node_->get_clock()->now();

  active_goal_.reset();

  auto gh_future = client_->async_send_goal(goal);
  if (gh_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
  {
    out.status = ExecStatus::TIMEOUT;
    out.message = "MoveItExecutor: timeout waiting for goal handle";
    return out;
  }

  active_goal_ = gh_future.get();
  if (!active_goal_)
  {
    out.status = ExecStatus::FAILURE;
    out.message = "MoveItExecutor: goal rejected";
    return out;
  }

  auto result_future = client_->async_get_result(active_goal_);

  const auto wait_dur = std::chrono::seconds(60);

  if (result_future.wait_for(wait_dur) != std::future_status::ready)
  {
    cancel();
    out.status = ExecStatus::TIMEOUT;
    out.message = "MoveItExecutor: timeout waiting for execution result";
    return out;
  }

  const auto& wrapped = result_future.get();

  if (wrapped.code == rclcpp_action::ResultCode::CANCELED)
  {
    out.status = ExecStatus::CANCELED;
    out.message = "MoveItExecutor: execution canceled";
  }
  else if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    out.status = ExecStatus::FAILURE;
    out.message = "MoveItExecutor: action result code != SUCCEEDED";
  }
  else
  {
    out.status = ExecStatus::SUCCESS;
    out.message = "MoveItExecutor: execution success";
  }

  const auto t_end = node_->get_clock()->now();
  out.execution_time_sec = (t_end - t_start).seconds();

  return out;
}

void MoveItExecutor::cancel()
{
  if (!client_ || !active_goal_)
  {
    return;
  }

  (void)client_->async_cancel_goal(active_goal_);
}

}  // namespace rlc_executive