#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

namespace rlc_executive
{

class RlcAlwaysSuccess final : public BT::SyncActionNode
{
public:
  RlcAlwaysSuccess(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace rlc_executive

namespace rlc_executive
{

void registerAlwaysSuccessNode(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<RlcAlwaysSuccess>("RlcAlwaysSuccess");
}

}  // namespace rlc_executive
