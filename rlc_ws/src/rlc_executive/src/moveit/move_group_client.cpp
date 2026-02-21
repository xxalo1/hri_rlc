#include "rlc_executive/moveit/move_group_client.hpp"

#include <chrono>
#include <future>
#include <string>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace rlc_executive
{

MoveGroupClient::MoveGroupClient(rclcpp::Node::SharedPtr& node, const ExecConfig& cfg)
  : node_(node.get()), cfg_(&cfg)
{
  client_ = rclcpp_action::create_client<MoveGroup>(node_, cfg_->move_group_action_name);
}

PlanResult MoveGroupClient::plan(const moveit_msgs::msg::MotionPlanRequest& req,
                                 const PlanningProfile& profile)
{
  PlanResult out;

  if (!client_)
  {
    out.status = PlanStatus::FAILURE;
    out.message = "MoveGroupClient: client_ is null";
    return out;
  }

  if (!client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    out.status = PlanStatus::FAILURE;
    out.message = "MoveGroupClient: MoveGroup action server not available: " +
                  cfg_->move_group_action_name;
    return out;
  }

  moveit_msgs::msg::MotionPlanRequest req_mut = req;
  req_mut.pipeline_id = profile.pipeline_id;
  req_mut.planner_id = profile.planner_id;
  req_mut.allowed_planning_time = profile.allowed_planning_time_sec;
  req_mut.num_planning_attempts = profile.num_planning_attempts;

  MoveGroup::Goal goal;
  goal.request = req_mut;
  goal.planning_options.plan_only = true;

  const auto t_start = node_->get_clock()->now();

  auto gh_future = client_->async_send_goal(goal);
  if (gh_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
  {
    out.status = PlanStatus::TIMEOUT;
    out.message = "MoveGroupClient: timeout waiting for goal handle";
    return out;
  }

  const auto& goal_handle = gh_future.get();
  if (!goal_handle)
  {
    out.status = PlanStatus::FAILURE;
    out.message = "MoveGroupClient: goal rejected";
    return out;
  }

  auto result_future = client_->async_get_result(goal_handle);

  const double wait_sec = profile.allowed_planning_time_sec + 2.0;
  const auto wait_dur = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(wait_sec));

  if (result_future.wait_for(wait_dur) != std::future_status::ready)
  {
    (void)client_->async_cancel_goal(goal_handle);
    out.status = PlanStatus::TIMEOUT;
    out.message = "MoveGroupClient: timeout waiting for result";
    return out;
  }

  const auto& wrapped = result_future.get();

  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    out.status = PlanStatus::FAILURE;
    out.message = "MoveGroupClient: action result code != SUCCEEDED";
    return out;
  }

  const auto& res = wrapped.result;

  out.moveit_error_code = res->error_code.val;
  out.planning_time_sec = res->planning_time;
  out.trajectory = res->planned_trajectory;

  if (res->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    out.status = PlanStatus::SUCCESS;
    out.message = "MoveGroupClient: plan success";
  }
  else
  {
    out.status = PlanStatus::FAILURE;
    out.message = "MoveGroupClient: planning failed with MoveItErrorCodes.val=" +
                  std::to_string(res->error_code.val);
  }

  const auto t_end = node_->get_clock()->now();
  const double wall_sec = (t_end - t_start).seconds();
  if (out.planning_time_sec <= 0.0)
  {
    out.planning_time_sec = wall_sec;
  }

  return out;
}

}  // namespace rlc_executive