#pragma once

#include <memory>
#include <optional>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/logger.hpp>

#include "rlc_executive/bt_nodes/bt_utils.hpp"
#include "rlc_executive/core/types.hpp"

namespace rlc_executive
{

class RuntimeContext;

/**
 * @brief Executes a planned trajectory using the active TrajectoryExecutor backend.
 *
 * @details
 * This BT node reads a trajectory and planning profile from input ports, then starts
 * execution through the runtime context stored on the blackboard (`bb::RUNTIME_CONTEXT`).
 *
 * Input ports:
 * - `trajectory`: Trajectory to execute.
 * - `planning_profile`: Planning profile name (selects execution backend configuration).
 *
 * Output ports:
 * - `exec_result`: Execution result (status + message + error_code + timing).
 * - `error`: Convenience string for introspection; set to a human-readable failure
 * message on failure/cancel and cleared on success.
 * - `feedback`: Convenience string for introspection; updated while running (e.g.,
 * "executing").
 * - `elapsed_sec`: Convenience value for introspection; elapsed execution time [s] while
 * running and execution time [s] when complete.
 */
class ExecuteTrajectory final : public BT::StatefulActionNode
{
public:
  ExecuteTrajectory(const std::string& name, const BT::NodeConfig& config);

  /// @brief Declares the ports used by this BT node.
  /// @return Port list for XML registration.
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<const moveit_msgs::msg::RobotTrajectory>>(
          PortKeys::TRAJECTORY),
      BT::InputPort<std::string>(PortKeys::PLANNING_PROFILE),
      BT::OutputPort<ExecResult>(PortKeys::EXEC_RESULT),
      BT::OutputPort<std::string>(PortKeys::ERROR),
      BT::OutputPort<std::string>(PortKeys::FEEDBACK),
      BT::OutputPort<double>(PortKeys::ELAPSED_SEC),
    };
  }

  /**
   * @brief Names of BT ports used by ExecuteTrajectory.
   */
  struct PortKeys : bt_utils::DiagnosticPortKeys
  {
    static inline const std::string TRAJECTORY = "trajectory";
    static inline const std::string PLANNING_PROFILE = "planning_profile";
    static inline const std::string EXEC_RESULT = "exec_result";
  };

private:
  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

  BT::NodeStatus failWithError(const std::string& msg);

  std::shared_ptr<RuntimeContext> ctx_;

  std::string profile_name_;

  std::optional<rclcpp::Logger> logger_;
};

}  // namespace rlc_executive
