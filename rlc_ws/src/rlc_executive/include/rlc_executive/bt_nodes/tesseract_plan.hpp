#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/logger.hpp>

#include "rbt_types/math.hpp"
#include "rbt_types/objective_term.hpp"
#include "rbt_types/trajectory.hpp"

#include "rlc_executive/bt_nodes/bt_utils.hpp"

namespace rbt_planning
{
class TrajOptPlanner;
}

namespace rlc_executive
{
class RuntimeContext;

/**
 * @brief BehaviorTree node that plans a joint-space trajectory with TrajOpt.
 *
 * @details
 * This BT node snapshots the monitored Tesseract environment, constructs a
 * `rbt_planning::TrajOptPlanner`, and runs planning on a worker thread. The node stays
 * in `RUNNING` until the worker finishes, then writes the planned trajectory or a
 * failure diagnostic.
 */
class TesseractPlan final : public BT::StatefulActionNode
{
public:
  using RewardTerms = rbt_types::RewardTerms;
  using Trajectory = rbt_types::Trajectory;

  using JointVec = rbt_types::JointVec;
  using JointEndpoints = rbt_types::JointEndpoints;

  using TrajOptPlanner = rbt_planning::TrajOptPlanner;

  /**
   * @brief Constructs the Tesseract planning BT node.
   * @param[in] name Node instance name used by BehaviorTree.CPP.
   * @param[in] config BehaviorTree node configuration, including blackboard access.
   */
  TesseractPlan(const std::string& name, const BT::NodeConfig& config);

  /**
   * @brief Declares the ports consumed and produced by this BT node.
   * @return BehaviorTree port list containing the planning group, joint endpoints,
   * reward terms, trajectory sample period [s], planned trajectory, and standard
   * diagnostic outputs.
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(PortKeys::GROUP_NAME),
      BT::InputPort<RewardTerms>(PortKeys::REWARD_TERMS),
      BT::InputPort<JointEndpoints>(PortKeys::JOINT_ENDPOINTS),
      BT::InputPort<double>(PortKeys::TRAJECTORY_DT),

      BT::OutputPort<std::shared_ptr<const Trajectory>>(PortKeys::TRAJECTORY),
      BT::OutputPort<std::string>(PortKeys::ERROR),
      BT::OutputPort<std::string>(PortKeys::FEEDBACK),
      BT::OutputPort<double>(PortKeys::ELAPSED_SEC),
    };
  }

  /**
   * @brief Blackboard key names used by TesseractPlan.
   */
  struct PortKeys : bt_utils::DiagnosticPortKeys
  {
    static inline const std::string JOINT_ENDPOINTS = "joint_endpoints";
    static inline const std::string GROUP_NAME = "group_name";
    static inline const std::string TRAJECTORY_DT = "trajectory_dt";
    static inline const std::string TRAJECTORY = "trajectory";
    static inline const std::string REWARD_TERMS = "reward_terms";
  };

private:
  std::shared_ptr<TrajOptPlanner> initializePlanner(const std::string& group_name,
                                                    double trajectory_dt);

  void runPlannerWorker(JointEndpoints joint_endpoints, RewardTerms reward_terms);

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

  BT::NodeStatus failWithError(const std::string& msg, double elapsed_sec = 0.0);

  std::shared_ptr<RuntimeContext> ctx_;

  std::optional<rclcpp::Logger> logger_;

  std::mutex result_mutex_;
  std::jthread worker_thread_{};

  std::shared_ptr<const Trajectory> trajectory_;
  
  std::string error_;

  double elapsed_sec_{ 0.0 };

  bool worker_done_{ false };
  
  std::shared_ptr<TrajOptPlanner> planner_;
};

}  // namespace rlc_executive
