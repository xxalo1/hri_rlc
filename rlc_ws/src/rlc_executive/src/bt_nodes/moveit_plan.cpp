#include "rlc_executive/bt_nodes/moveit_plan.hpp"

#include <exception>
#include <memory>
#include <string>
#include <utility>

#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>

#include "rlc_executive/bt_nodes/bt_utils.hpp"
#include "rlc_executive/core/runtime_context.hpp"
#include "rlc_executive/moveit/move_group_client.hpp"

namespace rlc_executive
{
namespace
{

PlanSummary makePlanSummary(PlanStatus status, const std::string& message)
{
  PlanSummary out;
  out.status = status;
  out.message = message;
  return out;
}

}  // namespace

MoveItPlan::MoveItPlan(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::NodeStatus MoveItPlan::onStart()
{
  try
  {
    bt_utils::ensureContextAndLogger(*this, ctx_, logger_);

    const auto req =
        bt_utils::requireInput<std::shared_ptr<const moveit_msgs::msg::MotionPlanRequest>>(
            *this, PortKeys::REQUEST);
    if (!req)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(), "]: input port '",
                             PortKeys::REQUEST, "' is null");
    }

    profile_name_ =
        bt_utils::requireInput<std::string>(*this, PortKeys::PLANNING_PROFILE);

    const PlanningProfile* profile = ctx_->getPlanningProfile(profile_name_);
    if (!profile)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: unknown planning_profile '", profile_name_, "'");
    }

    // Clear previous output to avoid stale blackboard values.
    bt_utils::setOutput(*this, PortKeys::TRAJECTORY,
                        std::shared_ptr<const moveit_msgs::msg::RobotTrajectory>());

    RCLCPP_INFO(*logger_, "start planning_profile='%s' group='%s'", profile_name_.c_str(),
                req->group_name.c_str());

    std::string err;
    if (!ctx_->moveGroupClient().start(*req, *profile, &err))
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: failed to start planning: ", err);
    }

    return BT::NodeStatus::RUNNING;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

BT::NodeStatus MoveItPlan::onRunning()
{
  try
  {
    auto pr_opt = ctx_->moveGroupClient().takeResult();
    if (!pr_opt)
    {
      return BT::NodeStatus::RUNNING;
    }

    PlanResult pr = std::move(*pr_opt);

    PlanSummary out;
    out.status = pr.status;
    out.moveit_error_code = pr.moveit_error_code;
    out.message = std::move(pr.message);
    out.planning_time_sec = pr.planning_time_sec;
    bt_utils::setOutput(*this, PortKeys::PLAN_RESULT, out);

    if (pr.status == PlanStatus::SUCCESS)
    {
      const std::shared_ptr<const moveit_msgs::msg::RobotTrajectory> traj =
          std::make_shared<moveit_msgs::msg::RobotTrajectory>(std::move(pr.trajectory));
      bt_utils::setOutput(*this, PortKeys::TRAJECTORY, traj);

      const std::size_t traj_points = traj->joint_trajectory.points.size();
      const std::size_t traj_joints = traj->joint_trajectory.joint_names.size();
      RCLCPP_INFO(*logger_, "SUCCESS (%.3fs) traj_points=%zu traj_joints=%zu",
                  out.planning_time_sec, traj_points, traj_joints);

      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(*logger_, "FAILURE (code=%d): %s", out.moveit_error_code,
                 out.message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

void MoveItPlan::onHalted()
{
  RCLCPP_WARN(*logger_, "halt requested, canceling");
  ctx_->moveGroupClient().requestCancel();
}

BT::NodeStatus MoveItPlan::failWithError(const std::string& msg)
{
  setOutput(PortKeys::PLAN_RESULT, makePlanSummary(PlanStatus::FAILURE, msg));
  setOutput(PortKeys::TRAJECTORY,
            std::shared_ptr<const moveit_msgs::msg::RobotTrajectory>());
  if (logger_)
  {
    RCLCPP_ERROR(*logger_, "%s", msg.c_str());
  }
  return BT::NodeStatus::FAILURE;
}

void registerMoveItPlanNode(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<MoveItPlan>("MoveItPlan");
}

}  // namespace rlc_executive
