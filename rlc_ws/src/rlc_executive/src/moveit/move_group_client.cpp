#include "rlc_executive/moveit/move_group_client.hpp"

#include <chrono>
#include <string>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace rlc_executive
{

MoveGroupClient::MoveGroupClient(rclcpp::Node& node, const ExecConfig& cfg)
  : node_(&node), cfg_(&cfg)
{
  client_ = rclcpp_action::create_client<MoveGroup>(node_, cfg_->move_group_action_name);
}

void MoveGroupClient::handleGoalResponse(std::uint64_t seq, const GoalHandle::SharedPtr& goal_handle)
{
  std::lock_guard<std::mutex> lock(mtx_);

  // If preempted, cancel to avoid orphan work.
  if (seq != active_seq_)
  {
    if (goal_handle && client_)
    {
      (void)client_->async_cancel_goal(goal_handle);
    }
    return;
  }

  if (!goal_handle)
  {
    PlanResult out;
    out.status = PlanStatus::FAILURE;
    out.message = "MoveGroupClient: goal rejected";

    active_ = false;
    active_goal_.reset();
    result_ = out;

    last_feedback_text_ = "goal_rejected";
    last_feedback_time_ = node_->get_clock()->now();
    return;
  }

  active_goal_ = goal_handle;
  last_feedback_text_ = "goal_accepted";
  last_feedback_time_ = node_->get_clock()->now();
}

void MoveGroupClient::handleFeedback(std::uint64_t seq)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (seq != active_seq_)
  {
    return;
  }

  // MoveGroup feedback is limited; still expose that planning is ongoing.
  last_feedback_text_ = "planning";
  last_feedback_time_ = node_->get_clock()->now();
}

void MoveGroupClient::handleResult(std::uint64_t seq, const GoalHandle::WrappedResult& wrapped)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (seq != active_seq_)
  {
    return;
  }

  PlanResult out;

  if (wrapped.code == rclcpp_action::ResultCode::CANCELED)
  {
    out.status = PlanStatus::FAILURE;
    out.message = "MoveGroupClient: planning canceled";
    last_feedback_text_ = "canceled";
  }
  else if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    out.status = PlanStatus::FAILURE;
    out.message = "MoveGroupClient: action result code != SUCCEEDED";
    last_feedback_text_ = "failed";
  }
  else
  {
    const auto& res = wrapped.result;

    out.moveit_error_code = res->error_code.val;
    out.planning_time_sec = res->planning_time;
    out.trajectory = res->planned_trajectory;

    if (res->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      out.status = PlanStatus::SUCCESS;
      out.message = "MoveGroupClient: plan success";
      last_feedback_text_ = "done";
    }
    else
    {
      out.status = PlanStatus::FAILURE;
      out.message = "MoveGroupClient: planning failed with MoveItErrorCodes.val=" +
                    std::to_string(res->error_code.val);
      last_feedback_text_ = "failed";
    }
  }

  active_ = false;
  active_goal_.reset();
  result_ = out;

  last_feedback_time_ = node_->get_clock()->now();
}

bool MoveGroupClient::start(const moveit_msgs::msg::MotionPlanRequest& req,
                            const PlanningProfile& profile, std::string* error_msg)
{
  if (!client_)
  {
    if (error_msg)
    {
      *error_msg = "MoveGroupClient: client_ is null";
    }
    return false;
  }

  // Non-blocking readiness check.
  if (!client_->wait_for_action_server(std::chrono::milliseconds(1000)))
  {
    if (error_msg)
    {
      *error_msg = "MoveGroupClient: MoveGroup action server not ready: " +
                   cfg_->move_group_action_name;
    }
    return false;
  }

  // Apply profile to request.
  moveit_msgs::msg::MotionPlanRequest req_mut = req;
  req_mut.pipeline_id = profile.pipeline_id;
  req_mut.planner_id = profile.planner_id;
  req_mut.allowed_planning_time = profile.allowed_planning_time_sec;
  req_mut.num_planning_attempts = profile.num_planning_attempts;

  MoveGroup::Goal goal;
  goal.request = req_mut;
  goal.planning_options.plan_only = true;

  const auto now = node_->get_clock()->now();

  std::uint64_t seq = 0;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    ++active_seq_;
    seq = active_seq_;

    active_ = true;
    active_goal_.reset();
    result_.reset();

    start_time_ = now;
    last_feedback_time_ = now;
    last_feedback_text_ = "goal_sent";
  }

  rclcpp_action::Client<MoveGroup>::SendGoalOptions opts;

  opts.goal_response_callback = [this, seq](const GoalHandle::SharedPtr& goal_handle) {
    handleGoalResponse(seq, goal_handle);
  };

  opts.feedback_callback =
      [this, seq](const GoalHandle::SharedPtr& /*goal_handle*/,
                  const std::shared_ptr<const MoveGroup::Feedback>& /*fb*/) {
        handleFeedback(seq);
      };

  opts.result_callback = [this, seq](const GoalHandle::WrappedResult& wrapped) {
    handleResult(seq, wrapped);
  };

  (void)client_->async_send_goal(goal, opts);
  return true;
}

bool MoveGroupClient::preemptAndStart(const moveit_msgs::msg::MotionPlanRequest& req,
                                      const PlanningProfile& profile,
                                      std::string* error_msg)
{
  requestCancel();
  return start(req, profile, error_msg);
}

void MoveGroupClient::requestCancel()
{
  if (!client_)
  {
    return;
  }

  GoalHandle::SharedPtr gh;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    gh = active_goal_;
    last_feedback_text_ = "cancel_requested";
    last_feedback_time_ = node_->get_clock()->now();
  }

  // If goal handle not yet available, cancel all goals (covers "sent but not accepted").
  if (gh)
  {
    (void)client_->async_cancel_goal(gh);
  }
  else
  {
    (void)client_->async_cancel_all_goals();
  }
}

bool MoveGroupClient::isActive() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return active_;
}

bool MoveGroupClient::hasResult() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return result_.has_value();
}

std::optional<PlanResult> MoveGroupClient::peekResult() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return result_;
}

std::optional<PlanResult> MoveGroupClient::takeResult()
{
  std::lock_guard<std::mutex> lock(mtx_);
  return std::move(result_);
}

PlanningFeedback MoveGroupClient::latestFeedback() const
{
  PlanningFeedback fb;

  std::lock_guard<std::mutex> lock(mtx_);
  fb.text = last_feedback_text_;

  if (active_)
  {
    const auto now = node_->get_clock()->now();
    fb.elapsed_sec = (now - start_time_).seconds();
  }
  else
  {
    fb.elapsed_sec = 0.0;
  }

  return fb;
}

}  // namespace rlc_executive
