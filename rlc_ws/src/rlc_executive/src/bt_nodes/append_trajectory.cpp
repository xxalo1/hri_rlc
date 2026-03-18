#include "rlc_executive/bt_nodes/append_trajectory.hpp"

#include <exception>
#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>

#include "rlc_executive/bt_nodes/bt_utils.hpp"

namespace rlc_executive
{

AppendTrajectory::AppendTrajectory(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus AppendTrajectory::tick()
{
  try
  {
    const std::shared_ptr<const Trajectory> trajectory =
        bt_utils::requireInput<std::shared_ptr<const Trajectory>>(*this,
                                                                  PortKeys::TRAJECTORY);
    if (!trajectory)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: input trajectory is null");
    }

    trajectory->validate();

    const std::shared_ptr<const TrajectorySet> existing_demos =
        bt_utils::requireInput<std::shared_ptr<const TrajectorySet>>(
            *this, PortKeys::DEMONSTRATIONS);

    auto demonstrations = std::make_shared<TrajectorySet>();
    if (existing_demos)
    {
      *demonstrations = *existing_demos;
    }
    demonstrations->push_back(*trajectory);

    const std::shared_ptr<const TrajectorySet> out = demonstrations;
    bt_utils::setOutput(*this, PortKeys::DEMONSTRATIONS, out);
    bt_utils::setOutput(*this, PortKeys::ERROR, std::string());
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

BT::NodeStatus AppendTrajectory::failWithError(const std::string& msg)
{
  bt_utils::setOutput(*this, PortKeys::ERROR, msg);
  return BT::NodeStatus::FAILURE;
}

void registerAppendTrajectoryNode(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<AppendTrajectory>("AppendTrajectory");
}

}  // namespace rlc_executive
