#include "rlc_executive/core/runtime_context.hpp"

#include <utility>

#include "rlc_executive/execution/moveit_executor.hpp"
#include "rlc_executive/moveit/moveit_client.hpp"
#include "rlc_executive/state/state_buffer.hpp"

namespace rlc_executive
{

RuntimeContext::RuntimeContext(rclcpp::Node& node, ExecConfig cfg)
  : node_(&node), cfg_(std::move(cfg))
{
  state_buffer_ = std::make_shared<StateBuffer>(node, cfg_);
  move_group_client_ = std::make_shared<MoveGroupClient>(node, cfg_);

  trajectory_executor_ = std::make_shared<MoveItExecutor>(node, cfg_);
}

rclcpp::Node& RuntimeContext::node() const
{
  return *node_;
}

const ExecConfig& RuntimeContext::config() const
{
  return cfg_;
}

StateBuffer& RuntimeContext::stateBuffer() const
{
  return *state_buffer_;
}

MoveGroupClient& RuntimeContext::moveGroupClient() const
{
  return *move_group_client_;
}

TrajectoryExecutor& RuntimeContext::trajectoryExecutor() const
{
  return *trajectory_executor_;
}

const PlanningProfile*
RuntimeContext::getPlanningProfile(const std::string& name) const noexcept
{
  const auto it = cfg_.planning_profiles.find(name);
  if (it == cfg_.planning_profiles.end())
  {
    return nullptr;
  }
  return &it->second;
}

const PlanningProfile* RuntimeContext::getDefaultPlanningProfile() const noexcept
{
  if (cfg_.default_planning_profile.empty())
  {
    return nullptr;
  }
  return getPlanningProfile(cfg_.default_planning_profile);
}

}  // namespace rlc_executive