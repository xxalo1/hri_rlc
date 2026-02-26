#include "rlc_executive/bt_nodes/load_pose.hpp"

#include <exception>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>

#include "rlc_executive/bt_nodes/bt_utils.hpp"
#include "rlc_executive/core/runtime_context.hpp"

namespace rlc_executive
{
namespace
{

void declareIfMissing(rclcpp::Node& node, const std::string& name,
                      const rclcpp::ParameterValue& def)
{
  if (!node.has_parameter(name))
  {
    (void)node.declare_parameter(name, def);
  }
}

}  // namespace

LoadPose::LoadPose(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus LoadPose::tick()
{
  try
  {
    bt_utils::ensureContextAndLogger(*this, ctx_, logger_);

    const std::string prefix =
        bt_utils::requireInput<std::string>(*this, PortKeys::PARAM_PREFIX);

    rclcpp::Node& node = ctx_->node();

    const std::string frame_key = prefix + ".frame_id";
    const std::string pos_key = prefix + ".position";
    const std::string quat_key = prefix + ".orientation_xyzw";

    declareIfMissing(node, frame_key,
                     rclcpp::ParameterValue(std::string("base_link")));
    declareIfMissing(node, pos_key,
                     rclcpp::ParameterValue(std::vector<double>{ 0.0, 0.0, 0.0 }));
    declareIfMissing(
        node, quat_key,
        rclcpp::ParameterValue(std::vector<double>{ 0.0, 0.0, 0.0, 1.0 }));

    std::string frame_id;
    std::vector<double> position;
    std::vector<double> orientation_xyzw;

    if (!node.get_parameter(frame_key, frame_id))
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: missing parameter '", frame_key, "'");
    }
    if (!node.get_parameter(pos_key, position))
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: missing parameter '", pos_key, "'");
    }
    if (!node.get_parameter(quat_key, orientation_xyzw))
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: missing parameter '", quat_key, "'");
    }

    if (position.size() != 3)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: parameter '", pos_key, "' must be length 3");
    }
    if (orientation_xyzw.size() != 4)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: parameter '", quat_key, "' must be length 4");
    }

    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = frame_id;
    out.header.stamp = node.get_clock()->now();

    out.pose.position.x = position[0];
    out.pose.position.y = position[1];
    out.pose.position.z = position[2];

    out.pose.orientation.x = orientation_xyzw[0];
    out.pose.orientation.y = orientation_xyzw[1];
    out.pose.orientation.z = orientation_xyzw[2];
    out.pose.orientation.w = orientation_xyzw[3];

    bt_utils::setOutput(*this, PortKeys::TARGET_POSE, out);
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

BT::NodeStatus LoadPose::failWithError(const std::string& msg)
{
  (void)setOutput(PortKeys::TARGET_POSE, geometry_msgs::msg::PoseStamped());
  if (logger_)
  {
    RCLCPP_ERROR(*logger_, "%s", msg.c_str());
  }
  return BT::NodeStatus::FAILURE;
}

void registerLoadPoseNode(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<LoadPose>("LoadPose");
}

}  // namespace rlc_executive

