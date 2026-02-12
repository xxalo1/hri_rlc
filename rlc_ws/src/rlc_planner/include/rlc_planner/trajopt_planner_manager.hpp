#pragma once

#include <rlc_planner/trajopt_planner_options.hpp>

#include <moveit/planning_interface/planning_interface.hpp>
#include <rclcpp/node.hpp>

#include <memory>
#include <string>
#include <vector>

namespace rlc_scene_bridge
{
class MoveItTesseractBridge;
}  // namespace rlc_scene_bridge

namespace rlc_planner
{

class TrajOptInterface;

class TrajOptPlannerManager final : public planning_interface::PlannerManager
{
public:
  using planning_interface::PlannerManager::getPlanningContext;

  TrajOptPlannerManager();

  ~TrajOptPlannerManager() override = default;

  bool initialize(const moveit::core::RobotModelConstPtr& model,
                  const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override;

  std::string getDescription() const override;

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override;

  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req,
                     moveit_msgs::msg::MoveItErrorCodes& error_code) const override;

  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;

private:
  TrajOptPlannerOptions declareAndLoadOptions(const std::string& pfx);

  bool
  initializeTesseractBridge(const rclcpp::Node::SharedPtr& node,
                            const rlc_scene_bridge::MoveItTesseractBridgeOptions& opt);

  void validateRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                       const planning_interface::MotionPlanRequest& req) const;

  moveit::core::RobotModelConstPtr model_;
  rclcpp::Node::SharedPtr node_;
  std::string parameter_namespace_;
  TrajOptPlannerOptions options_{};

  std::shared_ptr<rlc_scene_bridge::MoveItTesseractBridge> env_bridge_;
  std::shared_ptr<TrajOptInterface> trajopt_interface_;
};

}  // namespace rlc_planner
