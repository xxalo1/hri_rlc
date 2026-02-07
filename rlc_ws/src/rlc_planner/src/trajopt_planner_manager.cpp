#include <rlc_planner/trajopt_planner_manager.hpp>

#include <rlc_planner/trajopt_planning_context.hpp>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/utils/logger.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <memory>

namespace rlc_planner
{

rclcpp::Logger getLogger()
{
  return moveit::getLogger("rlc_planner.trajopt_planner_manager");
}

constexpr char kPlannerId[] = "rlc_trajopt";
constexpr char kPlannerDescription[] = "TrajOpt";

using Self = TrajOptPlannerManager;
Self::TrajOptPlannerManager() = default;

bool Self::initialize(const moveit::core::RobotModelConstPtr& model,
                      const rclcpp::Node::SharedPtr& node,
                      const std::string& parameter_namespace)
{
  model_ = model;
  node_ = node;
  parameter_namespace_ = parameter_namespace;

  if (!model_)
  {
    RCLCPP_ERROR(getLogger(), "initialize() failed: RobotModel is null");
    return false;
  }
  if (!node_)
  {
    RCLCPP_ERROR(getLogger(), "initialize() failed: Node is null");
    return false;
  }

  rlc_scene_bridge::Options opt{};

  const std::string pfx =
      parameter_namespace_.empty() ? std::string{} : (parameter_namespace_ + ".");

  node_->declare_parameter<std::string>(pfx + "tesseract_monitor_namespace",
                                        opt.monitor_namespace);
  node_->get_parameter(pfx + "tesseract_monitor_namespace", opt.monitor_namespace);

  env_bridge_ = std::make_shared<rlc_scene_bridge::MoveItTesseractBridge>(
      *node_, opt);

  return true;
}

std::string Self::getDescription() const
{
  return kPlannerDescription;
}

void Self::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();
  algs.emplace_back(kPlannerId);
}

bool Self::canServiceRequest(const planning_interface::MotionPlanRequest& req) const
{
  if (!model_)
  {
    return false;
  }
  if (!model_->hasJointModelGroup(req.group_name))
  {
    return false;
  }

  const bool planner_id_ok = req.planner_id.empty() || (req.planner_id == kPlannerId);
  if (!planner_id_ok)
  {
    return false;
  }

  return req.trajectory_constraints.constraints.empty();
}

planning_interface::PlanningContextPtr
Self::getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                         const planning_interface::MotionPlanRequest& req,
                         moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  if (!node_ || !model_)
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    error_code.message = "TrajOptPlannerManager is not initialized";
    error_code.source = "rlc_planner";
    return nullptr;
  }
  if (!planning_scene)
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    error_code.message = "PlanningScene is null";
    error_code.source = "rlc_planner";
    return nullptr;
  }
  if (!model_->hasJointModelGroup(req.group_name))
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    error_code.message = "Unknown group_name: '" + req.group_name + "'";
    error_code.source = "rlc_planner";
    return nullptr;
  }
  if (!canServiceRequest(req))
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    error_code.message = "Request not supported by planner_id='" + req.planner_id + "'";
    error_code.source = "rlc_planner";
    return nullptr;
  }

  auto env = env_bridge_->envSnapshot();
  const std::string context_name =
      req.planner_id.empty() ? std::string{ kPlannerId } : req.planner_id;
  auto context = std::make_shared<TrajOptPlanningContext>(context_name, req.group_name,
                                                          node_, parameter_namespace_);
  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(req);

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  error_code.message = "";
  error_code.source = "rlc_planner";
  return context;
}

}  // namespace rlc_planner

PLUGINLIB_EXPORT_CLASS(rlc_planner::TrajOptPlannerManager,
                       planning_interface::PlannerManager)