#pragma once

#include <memory>
#include <optional>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/logger.hpp>

#include "rlc_executive/core/types.hpp"

#include "rlc_executive/bt_nodes/bt_utils.hpp"
#include "rbt_irl/max_ent_irl.hpp"
#include "rbt_types/objective_term.hpp"
namespace rlc_executive
{

class RuntimeContext;

class FitMaxEntIRL final : public BT::StatefulActionNode
{
public:
  FitMaxEntIRL(const std::string& name, const BT::NodeConfig& config);

  /// @brief Declares the ports used by this BT node.
  /// @return Port list for XML registration.
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<const moveit_msgs::msg::RobotTrajectory>>(
          PortKeys::TRAJECTORY),
      BT::OutputPort<rbt_types::RewardTerms>(PortKeys::REWARD_TERMS),
    };
  }

  /**
   * @brief Names of BT ports used by FitMaxEntIRL.
   */
  struct PortKeys : bt_utils::DiagnosticPortKeys
  {
    static inline const std::string TRAJECTORY = "trajectory";
    static inline const std::string REWARD_TERMS = "reward_terms";
  };

private:
  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

  BT::NodeStatus failWithError(const std::string& msg);

  std::shared_ptr<RuntimeContext> ctx_;

  std::optional<rclcpp::Logger> logger_;
};

}  // namespace rlc_executive
