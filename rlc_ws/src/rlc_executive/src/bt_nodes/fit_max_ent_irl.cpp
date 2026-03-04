#include "rlc_executive/bt_nodes/fit_max_ent_irl.hpp"

#include <exception>
#include <string>

#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>

#include "rlc_executive/bt_nodes/bt_utils.hpp"
#include "rlc_executive/core/runtime_context.hpp"
namespace rlc_executive
{
namespace
{

}  // namespace

FitMaxEntIRL::FitMaxEntIRL(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::NodeStatus FitMaxEntIRL::onStart()
{
  try
  {
    bt_utils::ensureContextAndLogger(*this, ctx_, logger_);

    const auto traj =
        bt_utils::requireInput<std::shared_ptr<const moveit_msgs::msg::RobotTrajectory>>(
            *this, PortKeys::TRAJECTORY);
    if (!traj)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(), "]: input port '",
                             PortKeys::TRAJECTORY, "' is null");
    }

    // rbt_irl::MaxEntIRL irl;

    // bt_utils::setRunningDiagnostics(*this, fb.text, fb.elapsed_sec);

    return BT::NodeStatus::RUNNING;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

BT::NodeStatus FitMaxEntIRL::onRunning()
{
  try
  {

    return BT::NodeStatus::RUNNING;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

void FitMaxEntIRL::onHalted()
{
  RCLCPP_WARN(*logger_, "halt requested, canceling");
}

BT::NodeStatus FitMaxEntIRL::failWithError(const std::string& msg)
{
  bt_utils::setFailureDiagnostics(*this, msg, 0.0);
  if (logger_)
  {
    RCLCPP_ERROR(*logger_, "%s", msg.c_str());
  }
  return BT::NodeStatus::FAILURE;
}

void registerFitMaxEntIRLNode(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<FitMaxEntIRL>("FitMaxEntIRL");
}

}  // namespace rlc_executive
