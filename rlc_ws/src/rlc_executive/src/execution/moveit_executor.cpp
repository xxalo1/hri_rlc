#include "rlc_executive/execution/moveit_executor.hpp"

#include <chrono>
#include <string>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace rlc_executive
{

MoveItExecutor::MoveItExecutor(rclcpp::Node& node, const ExecConfig& cfg)
  : node_(&node), cfg_(&cfg)
{
  client_ = rclcpp_action::create_client<ExecuteTrajectory>(
      node_, cfg_->execute_traj_action_name);
}

void MoveItExecutor::setImmediateFailure(ExecResult out)
{
  std::lock_guard<std::mutex> lock(mtx_);
  active_ = false;
  active_goal_.reset();
  result_ = std::move(out);
}

void MoveItExecutor::handleGoalResponse(std::uint64_t seq, const GoalHandle::SharedPtr& gh)
{
  std::lock_guard<std::mutex> lock(mtx_);

  // If this is not the current active request anymore, cancel it to avoid orphan execution.
  if (seq != active_seq_)
  {
    if (gh && client_)
    {
      (void)client_->async_cancel_goal(gh);
    }
    return;
  }

  if (!gh)
  {
    ExecResult out;
    out.status = ExecStatus::FAILURE;
    out.message = "MoveItExecutor: goal rejected";
    active_ = false;
    active_goal_.reset();
    result_ = out;
    last_feedback_text_ = "goal_rejected";
    last_feedback_time_ = node_->get_clock()->now();
    return;
  }

  active_goal_ = gh;
  last_feedback_text_ = "goal_accepted";
  last_feedback_time_ = node_->get_clock()->now();
}

void MoveItExecutor::handleFeedback(std::uint64_t seq)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (seq != active_seq_)
  {
    return;
  }

  // ExecuteTrajectory feedback may be minimal; we still expose "executing".
  last_feedback_text_ = "executing";
  last_feedback_time_ = node_->get_clock()->now();
}

void MoveItExecutor::handleResult(std::uint64_t seq, const GoalHandle::WrappedResult& wrapped)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (seq != active_seq_)
  {
    return;
  }

  ExecResult out;

  if (wrapped.code == rclcpp_action::ResultCode::CANCELED)
  {
    out.status = ExecStatus::CANCELED;
    out.message = "MoveItExecutor: execution canceled";
    last_feedback_text_ = "canceled";
  }
  else if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    out.status = ExecStatus::FAILURE;
    out.message = "MoveItExecutor: action result code != SUCCEEDED";
    last_feedback_text_ = "failed";
  }
  else
  {
    // ExecuteTrajectory provides a MoveItErrorCodes in result->error_code
    const int code = (wrapped.result) ? wrapped.result->error_code.val : 0;
    out.error_code = code;

    if (code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      out.status = ExecStatus::SUCCESS;
      out.message = "MoveItExecutor: execution success";
      last_feedback_text_ = "done";
    }
    else
    {
      out.status = ExecStatus::FAILURE;
      out.message = "MoveItExecutor: execution returned MoveItErrorCodes.val=" +
                    std::to_string(code);
      last_feedback_text_ = "failed";
    }
  }

  const auto t_end = node_->get_clock()->now();
  out.execution_time_sec = (t_end - start_time_).seconds();

  active_ = false;
  active_goal_.reset();
  result_ = out;
  last_feedback_time_ = t_end;
}

bool MoveItExecutor::start(const moveit_msgs::msg::RobotTrajectory& traj,
                           const PlanningProfile& /*profile*/, std::string* error_msg)
{
  if (!client_)
  {
    if (error_msg)
    {
      *error_msg = "MoveItExecutor: client_ is null";
    }
    return false;
  }

  if (!client_->wait_for_action_server(std::chrono::milliseconds(0)))
  {
    if (error_msg)
    {
      *error_msg = "MoveItExecutor: ExecuteTrajectory action server not ready: " +
                   cfg_->execute_traj_action_name;
    }
    return false;
  }

  ExecuteTrajectory::Goal goal;
  goal.trajectory = traj;

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

  rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions opts;

  opts.goal_response_callback = [this, seq](const GoalHandle::SharedPtr& gh) {
    handleGoalResponse(seq, gh);
  };

  opts.feedback_callback =
      [this, seq](const GoalHandle::SharedPtr&,
                  const std::shared_ptr<const ExecuteTrajectory::Feedback>&) {
        handleFeedback(seq);
      };

  opts.result_callback = [this, seq](const GoalHandle::WrappedResult& wrapped) {
    handleResult(seq, wrapped);
  };

  (void)client_->async_send_goal(goal, opts);
  return true;
}

bool MoveItExecutor::preemptAndStart(const moveit_msgs::msg::RobotTrajectory& traj,
                                     const PlanningProfile& profile,
                                     std::string* error_msg)
{
  requestCancel();
  return start(traj, profile, error_msg);
}

void MoveItExecutor::requestCancel()
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

  if (gh)
  {
    (void)client_->async_cancel_goal(gh);
  }
  else
  {
    (void)client_->async_cancel_all_goals();
  }
}

bool MoveItExecutor::isActive() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return active_;
}

bool MoveItExecutor::hasResult() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return result_.has_value();
}

std::optional<ExecResult> MoveItExecutor::peekResult() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return result_;
}

std::optional<ExecResult> MoveItExecutor::takeResult()
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto out = result_;
  result_.reset();
  return out;
}

ExecutionFeedback MoveItExecutor::latestFeedback() const
{
  ExecutionFeedback fb;

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
