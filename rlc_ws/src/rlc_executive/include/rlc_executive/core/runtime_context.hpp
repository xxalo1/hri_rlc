#pragma once

#include <memory>
#include <string>

#include <rclcpp/node.hpp>

#include "rlc_executive/core/exec_config.hpp"

namespace tesseract_monitoring
{
class ROSEnvironmentMonitorInterface;
}

namespace rlc_executive
{

class StateBuffer;
class MoveGroupClient;
class TrajectoryExecutor;
class MoveItServoClient;
class TeleoperationSession;

class RuntimeContext final
{
public:
  RuntimeContext(rclcpp::Node& node, ExecConfig cfg);

  // Returns nullptr if not found.
  const PlanningProfile* getPlanningProfile(const std::string& name) const noexcept;

  // Convenience: returns default profile if set, otherwise nullptr.
  const PlanningProfile* getDefaultPlanningProfile() const noexcept;

  rclcpp::Node& node() const
  {
    return *node_;
  }

  const ExecConfig& config() const
  {
    return cfg_;
  }

  StateBuffer& stateBuffer() const
  {
    return *state_buffer_;
  }

  MoveGroupClient& moveGroupClient() const
  {
    return *move_group_client_;
  }

  TrajectoryExecutor& trajectoryExecutor() const
  {
    return *trajectory_executor_;
  }

  MoveItServoClient& moveItServoClient() const
  {
    return *moveit_servo_client_;
  }

  tesseract_monitoring::ROSEnvironmentMonitorInterface& envMonitorInterface() const
  {
    return *env_monitor_interface_;
  }

  TeleoperationSession& teleoperationSession() const
  {
    return *teleoperation_session_;
  }

private:
  rclcpp::Node* node_ = nullptr;  // non-owning
  ExecConfig cfg_;

  std::shared_ptr<StateBuffer> state_buffer_;
  std::shared_ptr<MoveGroupClient> move_group_client_;
  std::shared_ptr<MoveItServoClient> moveit_servo_client_;
  std::shared_ptr<TrajectoryExecutor> trajectory_executor_;
  std::shared_ptr<tesseract_monitoring::ROSEnvironmentMonitorInterface>
      env_monitor_interface_;
  std::shared_ptr<TeleoperationSession> teleoperation_session_;
};

}  // namespace rlc_executive
