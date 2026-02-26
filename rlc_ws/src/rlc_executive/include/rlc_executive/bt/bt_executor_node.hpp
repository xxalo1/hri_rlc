#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "rlc_executive/bt/tree_runner.hpp"
#include "rlc_executive/core/exec_config.hpp"

namespace BT
{
class Groot2Publisher;
}  // namespace BT

namespace rlc_executive
{

class RuntimeContext;

/**
 * @brief ROS 2 node that loads and ticks a BehaviorTree executive.
 *
 * @details
 * Parameters:
 * - `executor_config`: Path to executor YAML config (absolute or relative to package share).
 * - `profiles_config`: Path to planning profiles YAML config.
 * - `plugins`: Path to BT plugin list file (or a single shared library path).
 * - `default_tree`: Path to BT XML to load.
 * - `enable_groot2`: If true, starts a BT.CPP `Groot2Publisher` for live monitoring.
 * - `groot2_port`: TCP port used by `Groot2Publisher` (default: 1667).
 */
class BtExecutorNode final : public rclcpp::Node
{
public:
  explicit BtExecutorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// @brief Destructor.
  ~BtExecutorNode() override;

  /// @brief Loads configs, creates the BT, and starts ticking.
  void start();

private:
  void tick();

  std::string resolveSharePath(const std::string& subdir,
                               const std::string& filename) const;

  void onTreeCompleted(BT::NodeStatus status);

  ExecConfig cfg_;
  std::shared_ptr<RuntimeContext> ctx_;
  TreeRunner tree_runner_;

  std::unique_ptr<BT::Groot2Publisher> groot2_publisher_;

  rclcpp::TimerBase::SharedPtr tick_timer_;

  bool completed_{false};
};

}  // namespace rlc_executive
