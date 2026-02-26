#pragma once

#include <memory>
#include <optional>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/logger.hpp>

namespace rlc_executive
{

class RuntimeContext;

/**
 * @brief Loads a target pose from ROS parameters.
 *
 * @details
 * Parameters are read from a prefix namespace (e.g. `default_goal.*`), which can be
 * selected using the `param_prefix` input port.
 *
 * Expected parameters:
 * - `<prefix>.frame_id` (string)
 * - `<prefix>.position` (double[3]) in meters
 * - `<prefix>.orientation_xyzw` (double[4]) quaternion `[x y z w]`
 *
 * If any parameter is missing, it will be declared with a default value before being
 * read.
 *
 * Input ports:
 * - `param_prefix`: Parameter namespace prefix (default: `"default_goal"`).
 *
 * Output ports:
 * - `target_pose`: Pose loaded from parameters.
 */
class LoadPose final : public BT::SyncActionNode
{
public:
  LoadPose(const std::string& name, const BT::NodeConfig& config);

  /// @brief Declares the ports used by this BT node.
  /// @return Port list for XML registration.
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(PortKeys::PARAM_PREFIX, "default_goal"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(PortKeys::TARGET_POSE),
    };
  }

  BT::NodeStatus tick() override;

  /**
   * @brief Names of BT ports used by LoadPose.
   */
  struct PortKeys
  {
    static inline const std::string PARAM_PREFIX = "param_prefix";
    static inline const std::string TARGET_POSE = "target_pose";
  };

private:
  BT::NodeStatus failWithError(const std::string& msg);

  std::shared_ptr<RuntimeContext> ctx_;

  std::optional<rclcpp::Logger> logger_;
};

}  // namespace rlc_executive

