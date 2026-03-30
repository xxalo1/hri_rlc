#include "rlc_executive/core/runtime_context.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include "rlc_executive/execution/moveit_executor.hpp"
#include "rlc_executive/moveit/move_group_client.hpp"
#include "rlc_executive/servo/teleoperation_controller.hpp"
#include "rlc_executive/state/state_buffer.hpp"

#include "rlc_utils/tesseract/tesseract_utils.hpp"

namespace rlc_executive
{

RuntimeContext::RuntimeContext(rclcpp::Node& node, ExecConfig cfg)
  : node_(&node), cfg_(std::move(cfg))
{
  if (cfg_.monitor.monitor_namespace.empty())
  {
    throw std::invalid_argument(
        "RuntimeContext: monitor.monitor_namespace must not be empty");
  }
  if (cfg_.monitor.wait_timeout.count() < 0)
  {
    throw std::invalid_argument("RuntimeContext: monitor.wait_timeout must be >= 0");
  }
  if (!cfg_.planning.default_profile.empty() &&
      cfg_.planning.profiles.find(cfg_.planning.default_profile) ==
          cfg_.planning.profiles.end())
  {
    throw std::invalid_argument("RuntimeContext: planning.default_profile '" +
                                cfg_.planning.default_profile + "' was not loaded");
  }

  state_buffer_ = std::make_shared<StateBuffer>(node, cfg_.state);
  move_group_client_ = std::make_shared<MoveGroupClient>(node, cfg_.move_group);
  trajectory_executor_ = std::make_shared<MoveItExecutor>(node, cfg_.execution);
  teleoperation_controller_ =
      std::make_shared<TeleoperationController>(node, cfg_.teleoperation);

  env_monitor_interface_ = rlc_utils::tesseract_utils::makeMonitorInterfaceFromParentNode(
      node, node.get_logger(), cfg_.monitor.monitor_namespace, cfg_.monitor.wait_timeout,
      cfg_.monitor.env_name);
}

const PlanningProfile*
RuntimeContext::getPlanningProfile(const std::string& name) const noexcept
{
  const auto it = cfg_.planning.profiles.find(name);
  if (it == cfg_.planning.profiles.end())
  {
    return nullptr;
  }
  return &it->second;
}

const PlanningProfile* RuntimeContext::getDefaultPlanningProfile() const noexcept
{
  if (cfg_.planning.default_profile.empty())
  {
    return nullptr;
  }
  return getPlanningProfile(cfg_.planning.default_profile);
}

}  // namespace rlc_executive
