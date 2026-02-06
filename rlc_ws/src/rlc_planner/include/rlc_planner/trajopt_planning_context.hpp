#pragma once

#include <moveit/planning_interface/planning_interface.hpp>
#include <rclcpp/node.hpp>

#include <atomic>
#include <string>

namespace rlc_planner {

/**
 * @brief Request-scoped planning context for the `rlc_trajopt` planner.
 *
 * @details
 * Instances are created per request by TrajOptPlannerManager::getPlanningContext().
 *
 * @par Thread safety
 * Not thread-safe. Each instance is intended to be used by a single planning
 * request.
 */
class TrajOptPlanningContext final : public planning_interface::PlanningContext {
 public:
  /**
   * @brief Construct a planning context for a single request.
   * @param[in] name Context name (typically the resolved `planner_id`).
   * @param[in] group MoveIt joint model group name (`req.group_name`).
   * @param[in] node ROS node used for parameters and logging; must outlive this
   * context.
   * @param[in] parameter_namespace Planner parameter namespace.
   */
  TrajOptPlanningContext(const std::string& name, const std::string& group,
                         const rclcpp::Node::SharedPtr& node,
                         const std::string& parameter_namespace);

  ~TrajOptPlanningContext() override = default;
  void solve(planning_interface::MotionPlanResponse& res) override;

  void solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;

  void clear() override;

 private:
  rclcpp::Node::SharedPtr node_;
  std::string parameter_namespace_;
  std::atomic<bool> terminate_requested_{false};
};

}  // namespace rlc_planner
