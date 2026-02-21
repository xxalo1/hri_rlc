#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "rlc_executive/bt/tree_runner.hpp"
#include "rlc_executive/core/exec_config.hpp"

namespace rlc_executive
{

class RuntimeContext;

class BtExecutorNode final : public rclcpp::Node
{
public:
  explicit BtExecutorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void start();

private:
  void tickOnce();

  // Resolve a file under share/<package>/<subdir>/<filename>
  std::string resolveSharePath(const std::string& subdir,
                               const std::string& filename) const;

  ExecConfig cfg_;
  std::shared_ptr<RuntimeContext> ctx_;
  TreeRunner tree_runner_;

  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace rlc_executive