#include <behaviortree_cpp/bt_factory.h>

namespace rlc_executive
{

void registerAlwaysSuccessNode(BT::BehaviorTreeFactory& factory);
void registerAppendTrajectoryNode(BT::BehaviorTreeFactory& factory);
void registerBuildMotionPlanRequestNode(BT::BehaviorTreeFactory& factory);
void registerExecuteTrajectoryNode(BT::BehaviorTreeFactory& factory);
void registerLoadPoseNode(BT::BehaviorTreeFactory& factory);
void registerMoveItPlanNode(BT::BehaviorTreeFactory& factory);
void registerRecordServoDemoNode(BT::BehaviorTreeFactory& factory);
void registerTesseractPlanNode(BT::BehaviorTreeFactory& factory);

}  // namespace rlc_executive

BT_REGISTER_NODES(factory)
{
  rlc_executive::registerAlwaysSuccessNode(factory);
  rlc_executive::registerAppendTrajectoryNode(factory);
  rlc_executive::registerBuildMotionPlanRequestNode(factory);
  rlc_executive::registerExecuteTrajectoryNode(factory);
  rlc_executive::registerLoadPoseNode(factory);
  rlc_executive::registerMoveItPlanNode(factory);
  rlc_executive::registerRecordServoDemoNode(factory);
  rlc_executive::registerTesseractPlanNode(factory);
}
