#include <rlc_planner/trajopt_planner_manager.hpp>

#include <rlc_planner/exceptions.hpp>
#include <rlc_planner/trajopt_interface.hpp>
#include <rlc_planner/trajopt_planning_context.hpp>
#include <rlc_scene_bridge/moveit_tesseract_bridge.hpp>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/utils/logger.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <tesseract_monitoring/constants.h>
#include <tesseract_monitoring/environment_monitor_interface.h>
#include <tesseract_msgs/srv/get_environment_information.hpp>

#include <chrono>
#include <memory>
#include <rclcpp/exceptions/exceptions.hpp>

namespace rlc_planner
{

namespace
{
constexpr char TRAJOPT_PLANNER_ID[] = "rlc_trajopt";
constexpr char TRAJOPT_PLANNER_DESCRIPTION[] = "TrajOpt";

void setError(moveit_msgs::msg::MoveItErrorCodes& ec, int32_t error_val, std::string msg)
{
  ec.val = error_val;
  ec.message = std::move(msg);
  ec.source = TRAJOPT_PLANNER_ID;
}

rclcpp::Logger getLogger()
{
  return moveit::getLogger("rlc_planner.trajopt_planner_manager");
}

template <typename T>
T declareOrGetParameter(rclcpp::Node& node, const std::string& name,
                        const T& default_value)
{
  if (!node.has_parameter(name))
  {
    try
    {
      return node.declare_parameter<T>(name, default_value);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
    {
      // Fall through: parameter was declared by another component (e.g., automatic
      // declaration from overrides).
    }
  }

  T value = default_value;
  (void)node.get_parameter(name, value);
  return value;
}
}  // namespace

TrajOptPlannerManager::TrajOptPlannerManager() = default;

bool TrajOptPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
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

  const std::string param_prefix =
      parameter_namespace_.empty() ? std::string{} : (parameter_namespace_ + ".");

  options_ = declareAndLoadOptions(param_prefix);

  if (!initializeTesseractBridge(node_, options_.scene_bridge))
  {
    RCLCPP_ERROR(getLogger(), "initialize() failed: could not initialize scene bridge");
    return false;
  }

  try
  {
    trajopt_interface_ = std::make_shared<TrajOptInterface>(options_.trajopt);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(getLogger(),
                 "initialize() failed: could not create TrajOptInterface: %s", ex.what());
    return false;
  }

  return true;
}

TrajOptPlannerOptions TrajOptPlannerManager::declareAndLoadOptions(const std::string& pfx)
{
  TrajOptPlannerOptions opt{};

  // Scene bridge / Tesseract monitoring options
  const auto monitor_namespace_param = pfx + "scene_bridge.monitor_namespace";
  const auto env_name_param = pfx + "scene_bridge.env_name";
  const auto scene_topic_param = pfx + "scene_bridge.scene_topic";
  const auto get_scene_srv_param = pfx + "scene_bridge.get_scene_srv";
  const auto scene_components_param = pfx + "scene_bridge.scene_components";
  const auto srv_wait_ms_param = pfx + "scene_bridge.srv_wait_ms";
  const auto scene_qos_depth_param = pfx + "scene_bridge.scene_qos_depth";

  opt.scene_bridge.monitor_namespace =
      declareOrGetParameter<std::string>(*node_, monitor_namespace_param,
                                         opt.scene_bridge.monitor_namespace);
  // Tesseract environment name must match the environment monitor's ID (usually the
  // URDF robot name). Default to the MoveIt robot model name for convenience.
  opt.scene_bridge.env_name = declareOrGetParameter<std::string>(
      *node_, env_name_param, model_->getName());
  opt.scene_bridge.scene_topic =
      declareOrGetParameter<std::string>(*node_, scene_topic_param, opt.scene_bridge.scene_topic);
  opt.scene_bridge.get_scene_srv =
      declareOrGetParameter<std::string>(*node_, get_scene_srv_param,
                                         opt.scene_bridge.get_scene_srv);

  const int scene_components = declareOrGetParameter<int>(
      *node_, scene_components_param,
      static_cast<int>(opt.scene_bridge.scene_components));
  if (scene_components >= 0)
  {
    opt.scene_bridge.scene_components = static_cast<uint32_t>(scene_components);
  }
  else
  {
    RCLCPP_WARN(getLogger(),
                "scene_bridge.scene_components=%d is invalid; using default=%u",
                scene_components, opt.scene_bridge.scene_components);
  }

  const int srv_wait_ms = declareOrGetParameter<int>(
      *node_, srv_wait_ms_param, static_cast<int>(opt.scene_bridge.srv_wait.count()));
  if (srv_wait_ms >= 0)
  {
    opt.scene_bridge.srv_wait = std::chrono::milliseconds{ srv_wait_ms };
  }
  else
  {
    RCLCPP_WARN(getLogger(), "scene_bridge.srv_wait_ms=%d is invalid; using default=%ld",
                srv_wait_ms, static_cast<long>(opt.scene_bridge.srv_wait.count()));
  }

  const int scene_qos_depth = declareOrGetParameter<int>(
      *node_, scene_qos_depth_param, static_cast<int>(opt.scene_bridge.scene_qos_depth));
  if (scene_qos_depth > 0)
  {
    opt.scene_bridge.scene_qos_depth = static_cast<std::size_t>(scene_qos_depth);
  }
  else
  {
    RCLCPP_WARN(getLogger(),
                "scene_bridge.scene_qos_depth=%d is invalid; using default=%zu",
                scene_qos_depth, opt.scene_bridge.scene_qos_depth);
  }

  // Scene bridge environment sync options
  const auto allow_replace_param = pfx + "scene_bridge.env_sync.allow_replace";

  opt.scene_bridge.env_sync.allow_replace = declareOrGetParameter<bool>(
      *node_, allow_replace_param, opt.scene_bridge.env_sync.allow_replace);

  // TrajOpt interface options
  const auto default_num_steps_param = pfx + "trajopt.default_num_steps";
  const auto use_moveit_group_as_tesseract_manipulator_param =
      pfx + "trajopt.use_moveit_group_as_tesseract_manipulator";
  const auto fixed_tesseract_manipulator_group_param =
      pfx + "trajopt.fixed_tesseract_manipulator_group";
  const auto enable_seed_cache_param = pfx + "trajopt.enable_seed_cache";
  const auto enable_last_solution_cache_param =
      pfx + "trajopt.enable_last_solution_cache";

  opt.trajopt.default_num_steps = declareOrGetParameter<int>(
      *node_, default_num_steps_param, opt.trajopt.default_num_steps);
  opt.trajopt.use_moveit_group_as_tesseract_manipulator = declareOrGetParameter<bool>(
      *node_, use_moveit_group_as_tesseract_manipulator_param,
      opt.trajopt.use_moveit_group_as_tesseract_manipulator);
  opt.trajopt.fixed_tesseract_manipulator_group = declareOrGetParameter<std::string>(
      *node_, fixed_tesseract_manipulator_group_param,
      opt.trajopt.fixed_tesseract_manipulator_group);
  opt.trajopt.enable_seed_cache = declareOrGetParameter<bool>(
      *node_, enable_seed_cache_param, opt.trajopt.enable_seed_cache);
  opt.trajopt.enable_last_solution_cache = declareOrGetParameter<bool>(
      *node_, enable_last_solution_cache_param, opt.trajopt.enable_last_solution_cache);

  return opt;
}

bool TrajOptPlannerManager::initializeTesseractBridge(
    const rclcpp::Node::SharedPtr& node,
    const rlc_scene_bridge::MoveItTesseractBridgeOptions& opt)
{
  const std::string info_srv =
      "/" + opt.monitor_namespace +
      tesseract_monitoring::DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;
  auto info_client =
      node->create_client<tesseract_msgs::srv::GetEnvironmentInformation>(info_srv);

  if (!info_client->wait_for_service(std::chrono::seconds{ 3 }))
  {
    RCLCPP_ERROR(getLogger(),
                 "Tesseract monitor info service '%s' not available after 3 seconds",
                 info_srv.c_str());
    return false;
  }

  auto mon = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitorInterface>(
      node, opt.env_name);
  mon->addNamespace(opt.monitor_namespace);

  if (!mon->wait(std::chrono::seconds{ 3 }))
  {
    RCLCPP_ERROR(getLogger(), "Tesseract monitor not reachable under namespace '%s'",
                 opt.monitor_namespace.c_str());
    return false;
  }

  rlc_scene_bridge::MoveItTesseractBridge::MonitorInterfaceConstPtr mon_const = mon;
  try
  {
    env_bridge_ =
        std::make_shared<rlc_scene_bridge::MoveItTesseractBridge>(node, mon_const, opt);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(getLogger(), "Failed to construct MoveItTesseractBridge: %s", ex.what());
    return false;
  }
  return true;
}

std::string TrajOptPlannerManager::getDescription() const
{
  return TRAJOPT_PLANNER_DESCRIPTION;
}

void TrajOptPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();
  algs.emplace_back(TRAJOPT_PLANNER_ID);
}

bool TrajOptPlannerManager::canServiceRequest(
    const planning_interface::MotionPlanRequest& req) const
{
  if (!model_)
  {
    return false;
  }
  if (!model_->hasJointModelGroup(req.group_name))
  {
    return false;
  }

  const bool planner_id_ok =
      req.planner_id.empty() || (req.planner_id == TRAJOPT_PLANNER_ID);
  return planner_id_ok;
}

planning_interface::PlanningContextPtr TrajOptPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  try
  {
    validateRequest(planning_scene, req);
  }
  catch (const PlanningError& ex)
  {
    setError(error_code, ex.code(), ex.what());
    return nullptr;
  }
  catch (const std::exception& ex)
  {
    setError(error_code, moveit_msgs::msg::MoveItErrorCodes::FAILURE, ex.what());
    return nullptr;
  }

  TrajOptPlanningContext::EnvironmentPtr env_snapshot;
  env_snapshot = env_bridge_->envSnapshot();
  if (!env_snapshot)
  {
    setError(error_code, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED,
             "Failed to snapshot Tesseract environment");
    return nullptr;
  }

  const std::string context_name =
      req.planner_id.empty() ? std::string{ TRAJOPT_PLANNER_ID } : req.planner_id;

  auto context = std::make_shared<TrajOptPlanningContext>(
      context_name, req.group_name, trajopt_interface_, std::move(env_snapshot));
  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(req);

  setError(error_code, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, "");
  return context;
}

void TrajOptPlannerManager::validateRequest(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req) const
{
  if (!planning_scene)
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                        "PlanningScene is null");
  }
  if (!canServiceRequest(req))
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED,
                        "Request not supported by planner_id='" + req.planner_id + "'");
  }
  if (!env_bridge_)
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                        "Tesseract bridge is not initialized");
  }
  if (!env_bridge_->isSynchronized())
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED,
                        "Tesseract environment is not synchronized");
  }
  if (!trajopt_interface_)
  {
    throw PlanningError(moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                        "TrajOpt interface is not initialized");
  }
}

}  // namespace rlc_planner

PLUGINLIB_EXPORT_CLASS(rlc_planner::TrajOptPlannerManager,
                       planning_interface::PlannerManager)
