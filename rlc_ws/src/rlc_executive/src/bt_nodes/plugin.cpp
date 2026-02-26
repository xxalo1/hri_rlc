#include <behaviortree_cpp/bt_factory.h>

namespace rlc_executive
{

void registerAlwaysSuccessNode(BT::BehaviorTreeFactory& factory);
void registerBuildMotionPlanRequestNode(BT::BehaviorTreeFactory& factory);
void registerExecuteTrajectoryNode(BT::BehaviorTreeFactory& factory);
void registerLoadPoseNode(BT::BehaviorTreeFactory& factory);
void registerMoveItPlanNode(BT::BehaviorTreeFactory& factory);

}  // namespace rlc_executive

BT_REGISTER_NODES(factory)
{
  rlc_executive::registerAlwaysSuccessNode(factory);
  rlc_executive::registerBuildMotionPlanRequestNode(factory);
  rlc_executive::registerExecuteTrajectoryNode(factory);
  rlc_executive::registerLoadPoseNode(factory);
  rlc_executive::registerMoveItPlanNode(factory);
}

