#pragma once

#include <moveit/planning_interface/planning_interface.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <vector>

namespace rlc_planner {

/**
 * @brief MoveIt PlannerManager plugin entrypoint for TrajOpt.
 */
class TrajOptPlannerManager final : public planning_interface::PlannerManager {
 public:
  using planning_interface::PlannerManager::getPlanningContext;

  /**
   * @brief Construct an uninitialized planner manager.
   */
  TrajOptPlannerManager();

  /**
   * @brief Destroy the planner manager.
   */
  ~TrajOptPlannerManager() override = default;

  /**
   * @brief Initialize the planner plugin for a specific robot model and node.
   * @param[in] model MoveIt robot model that this plugin will plan for.
   * @param[in] node ROS node used for parameters, logging, and time.
   * @param[in] parameter_namespace Namespace for parameters belonging to this
   * planner plugin.
   * @return True on successful initialization.
   */
  bool initialize(const moveit::core::RobotModelConstPtr& model,
                  const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override;

  /**
   * @brief Get a short description string for this planner plugin.
   * @return Planner description.
   */
  std::string getDescription() const override;

  /**
   * @brief Get the names of supported planner IDs for this plugin.
   * @param[out] algs Supported `planner_id` values.
   */
  void getPlanningAlgorithms(std::vector<std::string>& algs) const override;

  /**
   * @brief Construct a request-scoped planning context.
   * @param[in] planning_scene Planning scene snapshot to plan in.
   * @param[in] req MoveIt motion planning request.
   * @param[out] error_code Error code filled on failure.
   * @return Planning context instance, or nullptr on failure.
   */
  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& planning_scene,
      const planning_interface::MotionPlanRequest& req,
      moveit_msgs::msg::MoveItErrorCodes& error_code) const override;

  /**
   * @brief Check whether this planner can service the given request.
   * @param[in] req MoveIt motion planning request.
   * @return True if the request can be handled.
   */
  bool canServiceRequest(
      const planning_interface::MotionPlanRequest& req) const override;

 private:
  moveit::core::RobotModelConstPtr model_;
  rclcpp::Node::SharedPtr node_;
  std::string parameter_namespace_;
};

}  // namespace rlc_planner
