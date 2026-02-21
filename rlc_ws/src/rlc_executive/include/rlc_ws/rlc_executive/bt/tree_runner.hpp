#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/tree.h>

#include "rlc_executive/core/types.hpp"

namespace rlc_executive
{

class RuntimeContext;

class TreeRunner final
{
public:
  TreeRunner() = default;

  void configure(const std::string& plugins_xml_path);

  void loadTreeFromXmlFile(const std::string& tree_xml_path,
                           const std::shared_ptr<RuntimeContext>& ctx);

  BT::NodeStatus tickOnce();

  void haltTree();

  void reset();

  BT::Blackboard::Ptr blackboard() const;

private:
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;

  std::string last_tree_xml_path_;
  std::string last_plugins_xml_path_;
};

}  // namespace rlc_executive