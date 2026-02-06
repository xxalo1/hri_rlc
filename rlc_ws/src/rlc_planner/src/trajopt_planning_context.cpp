#include <moveit/utils/logger.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <rlc_planner/trajopt_planning_context.hpp>

namespace rlc_planner {
namespace {

rclcpp::Logger getLogger() {
  return moveit::getLogger("rlc_planner.trajopt_planning_context");
}

}  // namespace

TrajOptPlanningContext::TrajOptPlanningContext(
    const std::string& name, const std::string& group,
    const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace)
    : planning_interface::PlanningContext(name, group),
      node_(node),
      parameter_namespace_(parameter_namespace) {
  if (!node_) {
    RCLCPP_WARN(getLogger(),
                "TrajOptPlanningContext constructed with a null Node");
  }
}

void TrajOptPlanningContext::solve(
    planning_interface::MotionPlanResponse& res) {
  if (terminate_requested_.load()) {
    res.error_code = moveit::core::MoveItErrorCode(
        moveit_msgs::msg::MoveItErrorCodes::PREEMPTED, "Planning terminated",
        "rlc_planner");
    return;
  }

  res.planner_id =
      request_.planner_id.empty() ? getName() : request_.planner_id;
  res.planning_time = 0.0;
  res.error_code = moveit::core::MoveItErrorCode(
      moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED,
      "rlc_trajopt planning is not implemented yet", "rlc_planner");
}

void TrajOptPlanningContext::solve(
    planning_interface::MotionPlanDetailedResponse& res) {
  res.trajectory.clear();
  res.description.clear();
  res.processing_time.clear();

  res.planner_id =
      request_.planner_id.empty() ? getName() : request_.planner_id;

  if (terminate_requested_.load()) {
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
    res.error_code.message = "Planning terminated";
    res.error_code.source = "rlc_planner";
    return;
  }

  res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
  res.error_code.message = "rlc_trajopt planning is not implemented yet";
  res.error_code.source = "rlc_planner";
}

bool TrajOptPlanningContext::terminate() {
  terminate_requested_.store(true);
  return true;
}

void TrajOptPlanningContext::clear() { terminate_requested_.store(false); }

}  // namespace rlc_planner
