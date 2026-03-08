#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/logger.hpp>

#include "rbt_types/math.hpp"
#include "rbt_irl/max_ent_irl.hpp"
#include "rbt_types/feature.hpp"
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
 * @brief BehaviorTree node that fits reward terms from a demonstrated trajectory.
 */
class FitMaxEntIRL final : public BT::StatefulActionNode
{
public:
  using RewardTerms = rbt_types::RewardTerms;
  using Trajectory = rbt_types::Trajectory;
  using TrajectorySet = rbt_types::TrajectorySet;
  using Features = rbt_types::Features;

  using JointVec = rbt_types::JointVec;
  using WeightVec = rbt_types::WeightVec;

  using MaxEntIRL = rbt_irl::MaxEntIRL;
  using TrajOptPlanner = rbt_planning::TrajOptPlanner;

  /**
   * @brief Constructs the MaxEnt IRL BT node.
   * @param[in] name Node instance name used by BehaviorTree.CPP.
   * @param[in] config BehaviorTree node configuration, including blackboard access.
   */
  FitMaxEntIRL(const std::string& name, const BT::NodeConfig& config);

  /**
   * @brief Declares the ports consumed and produced by this BT node.
   * @return BehaviorTree port list containing the demonstrated trajectories, feature
   * set, optional manipulator group, fitted reward terms, and standard diagnostic
   * outputs.
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(PortKeys::GROUP_NAME, "", ""),
      BT::InputPort<std::shared_ptr<const TrajectorySet>>(PortKeys::DEMONSTRATIONS),
      BT::InputPort<Features>(PortKeys::FEATURES),
      BT::OutputPort<RewardTerms>(PortKeys::REWARD_TERMS),
      BT::OutputPort<std::string>(PortKeys::ERROR),
      BT::OutputPort<std::string>(PortKeys::FEEDBACK),
      BT::OutputPort<double>(PortKeys::ELAPSED_SEC),
    };
  }

  /**
   * @brief Blackboard key names used by FitMaxEntIRL.
   */
  struct PortKeys : bt_utils::DiagnosticPortKeys
  {
    static inline const std::string GROUP_NAME = "group_name";
    static inline const std::string DEMONSTRATIONS = "demonstrations";
    static inline const std::string FEATURES = "features";
    static inline const std::string REWARD_TERMS = "reward_terms";
  };

private:
  void initializeIRL(const Features& features, const std::string& group_name,
                     double trajectory_dt);

  MaxEntIRL::SamplerFn makeTrajectorySampler(const std::string& group_name,
                                             double trajectory_dt);

  void iterationCallback(const WeightVec& theta, const WeightVec& theta_prev,
                         std::chrono::milliseconds elapsed_time, int iteration);

  void runFitWorker(std::stop_token stop_token,
                    std::shared_ptr<const TrajectorySet> demo_trajs);

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

  BT::NodeStatus failWithError(const std::string& msg, double elapsed_sec = 0.0);

  struct ProgressState
  {
    std::string feedback;
    std::string error;
    double elapsed_sec{ 0.0 };
    int iteration{ 0 };
    bool completed{ false };
    bool succeeded{ false };
  };

  std::shared_ptr<RuntimeContext> ctx_;

  std::optional<rclcpp::Logger> logger_;

  std::optional<MaxEntIRL> irl_;
  Features features_;
  std::mutex progress_mutex_;
  ProgressState progress_{};
  std::jthread worker_thread_{};
};

}  // namespace rlc_executive
