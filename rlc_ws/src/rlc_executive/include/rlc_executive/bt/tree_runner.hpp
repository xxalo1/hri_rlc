#pragma once

#include <memory>
#include <string>
#include <chrono>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>

#include "rlc_executive/core/exec_config.hpp"

namespace rlc_executive
{

class RuntimeContext;

class TreeRunner final
{
public:
  TreeRunner() = default;

  void configure(const std::string& plugins_path_or_list_file);

  void loadTreeFromXmlFile(const std::string& tree_xml_path,
                           const std::shared_ptr<RuntimeContext>& ctx);

  BT::NodeStatus tick();

  BT::NodeStatus tickExactlyOnce();

  BT::NodeStatus tickOnce();

  BT::NodeStatus tickWhileRunning(std::chrono::milliseconds max_block);

  void haltTree();

  void reset();

  BT::Blackboard::Ptr blackboard() const
  {
    return blackboard_;
  }

  /**
   * @brief Returns the tick scheduling mode used by tick().
   * @return Tick scheduling mode.
   */
  TickOption tickOption() const
  {
    return tick_option_;
  }

  std::chrono::milliseconds tickWhileRunningMaxBlock() const
  {
    return tick_while_running_max_block_;
  }

  /**
   * @brief Sets the tick scheduling mode used by tick().
   * @param[in] mode Tick scheduling mode.
   */
  void setTickOption(TickOption mode)
  {
    tick_option_ = mode;
  }

  void setTickWhileRunningMaxBlock(std::chrono::milliseconds max_block)
  {
    tick_while_running_max_block_ = max_block;
  }

private:
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;

  std::string last_tree_xml_path_;
  std::string last_plugins_path_or_list_file_;
  TickOption tick_option_{ TickOption::ONCE_UNLESS_WOKEN_UP };
  std::chrono::milliseconds tick_while_running_max_block_{ 100 };
};

}  // namespace rlc_executive
