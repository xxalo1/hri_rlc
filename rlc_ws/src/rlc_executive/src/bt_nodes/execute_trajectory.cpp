#include "rlc_executive/bt_nodes/execute_trajectory.hpp"

#include <exception>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <tesseract_monitoring/environment_monitor_interface.h>

#include "rlc_executive/bt_nodes/bt_utils.hpp"
#include "rlc_executive/core/runtime_context.hpp"
#include "rlc_executive/execution/moveit_executor.hpp"

namespace rlc_executive
{
namespace
{
ExecResult makeExecResult(ExecStatus status, const std::string& message)
{
  ExecResult er;
  er.status = status;
  er.message = message;
  return er;
}

}  // namespace

ExecuteTrajectory::ExecuteTrajectory(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::NodeStatus ExecuteTrajectory::onStart()
{
  try
  {
    bt_utils::ensureContextAndLogger(*this, ctx_, logger_);

    const auto traj =
        bt_utils::requireInput<std::shared_ptr<const rbt_types::Trajectory>>(
            *this, PortKeys::TRAJECTORY);
    if (!traj)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(), "]: input port '",
                             PortKeys::TRAJECTORY, "' is null");
    }

    const std::string group_name =
        bt_utils::requireInput<std::string>(*this, PortKeys::GROUP_NAME);
    profile_name_ =
        bt_utils::requireInput<std::string>(*this, PortKeys::PLANNING_PROFILE);

    const PlanningProfile* profile = ctx_->getPlanningProfile(profile_name_);
    if (!profile)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: unknown planning_profile '", profile_name_, "'");
    }

    const std::string& monitor_namespace = ctx_->config().monitor.monitor_namespace;
    auto env_uptr = ctx_->envMonitorInterface().getEnvironment(monitor_namespace);
    if (!env_uptr)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: failed to snapshot Tesseract environment from namespace '",
                             monitor_namespace, "'");
    }

    const std::vector<std::string> joint_names = env_uptr->getGroupJointNames(group_name);
    const moveit_msgs::msg::RobotTrajectory moveit_traj =
        bt_utils::toRobotTrajectory(*traj, joint_names);

    const std::size_t traj_points = traj->length();
    const std::size_t traj_joints = traj->dim();

    RCLCPP_INFO(*logger_, "start planning_profile='%s' traj_points=%zu traj_joints=%zu",
                profile_name_.c_str(), traj_points, traj_joints);

    std::string err;
    if (!ctx_->trajectoryExecutor().start(moveit_traj, *profile, &err))
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: failed to start execution: ", err);
    }

    const ExecutionFeedback fb = ctx_->trajectoryExecutor().latestFeedback();
    bt_utils::setRunningDiagnostics(*this, fb.text, fb.elapsed_sec);

    return BT::NodeStatus::RUNNING;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

BT::NodeStatus ExecuteTrajectory::onRunning()
{
  try
  {
    const ExecutionFeedback fb = ctx_->trajectoryExecutor().latestFeedback();
    bt_utils::setRunningDiagnostics(*this, fb.text, fb.elapsed_sec);

    const auto er_opt = ctx_->trajectoryExecutor().takeResult();
    if (!er_opt)
    {
      return BT::NodeStatus::RUNNING;
    }

    const ExecResult& er = *er_opt;
    bt_utils::setOutput(*this, PortKeys::EXEC_RESULT, er);

    if (er.status == ExecStatus::SUCCESS)
    {
      bt_utils::setSuccessDiagnostics(*this, er.execution_time_sec);
      RCLCPP_INFO(*logger_, "SUCCESS (%.3fs)", er.execution_time_sec);
      return BT::NodeStatus::SUCCESS;
    }

    bt_utils::setFailureDiagnostics(*this, er.message, er.execution_time_sec);

    if (er.status == ExecStatus::CANCELED)
    {
      RCLCPP_WARN(*logger_, "CANCELED: %s", er.message.c_str());
    }
    else
    {
      RCLCPP_ERROR(*logger_, "FAILURE (code=%d): %s", er.error_code, er.message.c_str());
    }

    return BT::NodeStatus::FAILURE;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

void ExecuteTrajectory::onHalted()
{
  RCLCPP_WARN(*logger_, "halt requested, canceling");
  ctx_->trajectoryExecutor().requestCancel();
}

BT::NodeStatus ExecuteTrajectory::failWithError(const std::string& msg)
{
  (void)setOutput(PortKeys::EXEC_RESULT, makeExecResult(ExecStatus::FAILURE, msg));
  bt_utils::setFailureDiagnostics(*this, msg, 0.0);
  if (logger_)
  {
    RCLCPP_ERROR(*logger_, "%s", msg.c_str());
  }
  return BT::NodeStatus::FAILURE;
}

void registerExecuteTrajectoryNode(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<ExecuteTrajectory>("ExecuteTrajectory");
}

}  // namespace rlc_executive
