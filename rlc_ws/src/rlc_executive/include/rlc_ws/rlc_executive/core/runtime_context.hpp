#pragma once

#include <memory>
#include <string>

#include <rclcpp/node.hpp>

#include "rlc_executive/core/exec_config.hpp"

namespace rlc_executive
{

class StateBuffer;
class MoveGroupClient;
class TrajectoryExecutor;

class RuntimeContext final
{
public:
  RuntimeContext(rclcpp::Node& node, ExecConfig cfg);

  rclcpp::Node& node() const;

  const ExecConfig& config() const;

  StateBuffer& stateBuffer() const;
  MoveGroupClient& moveGroupClient() const;
  TrajectoryExecutor& trajectoryExecutor() const;

  // Returns nullptr if not found.
  const PlanningProfile* getPlanningProfile(const std::string& name) const noexcept;

  // Convenience: returns default profile if set, otherwise nullptr.
  const PlanningProfile* getDefaultPlanningProfile() const noexcept;

private:
  rclcpp::Node* node_ = nullptr;  // non-owning
  ExecConfig cfg_;

  std::shared_ptr<StateBuffer> state_buffer_;
  std::shared_ptr<MoveGroupClient> move_group_client_;
  std::shared_ptr<TrajectoryExecutor> trajectory_executor_;
};

}  // namespace rlc_executive