#pragma once

#include <memory>
#include <optional>
#include <string>

#include <behaviortree_cpp/action_node.h>

#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/logger.hpp>

#include "rlc_executive/core/types.hpp"

namespace rlc_executive
{

class RuntimeContext;

/**
 * @brief Starts MoveIt planning for a MotionPlanRequest.
 *
 * @details
 * This BT node reads a motion plan request and planning profile from input ports, then
 * starts planning through the runtime context stored on the blackboard
 * (`bb::RUNTIME_CONTEXT`).
 *
 * Input ports:
 * - `request`: Motion plan request to send to MoveIt.
 * - `planning_profile`: Planning profile name (selects planner pipeline and knobs).
 *
 * Output ports:
 * - `plan_result`: Planning result summary (status + message + error_code + timing).
 * - `trajectory`: Planned trajectory on success.
 */
class MoveItPlan final : public BT::StatefulActionNode
{
public:
  MoveItPlan(const std::string& name, const BT::NodeConfig& config);

  /// @brief Declares the ports used by this BT node.
  /// @return Port list for XML registration.
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<const moveit_msgs::msg::MotionPlanRequest>>(
          PortKeys::REQUEST),
      BT::InputPort<std::string>(PortKeys::PLANNING_PROFILE),
      BT::OutputPort<PlanSummary>(PortKeys::PLAN_RESULT),
      BT::OutputPort<std::shared_ptr<const moveit_msgs::msg::RobotTrajectory>>(
          PortKeys::TRAJECTORY),
    };
  }

  /**
   * @brief Names of BT ports used by MoveItPlan.
   */
  struct PortKeys
  {
    static inline const std::string REQUEST = "request";
    static inline const std::string PLANNING_PROFILE = "planning_profile";
    static inline const std::string PLAN_RESULT = "plan_result";
    static inline const std::string TRAJECTORY = "trajectory";
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
