#pragma once

#include <memory>
#include <optional>
#include <behaviortree_cpp/action_node.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <rclcpp/logger.hpp>
#include <string>
#include <vector>
#include "rlc_executive/bt_nodes/bt_utils.hpp"
namespace rlc_executive
{

class RuntimeContext;

/**
 * @brief Builds a MoveIt motion plan request from BT ports.
 *
 * @details
 * Goal input modes:
 * - Pose goal: provide `target_pose` (in `target_pose.header.frame_id`) and optionally `link_name`.
 * - Joint goal: provide both `joint_names` and `joint_positions`.
 *
 * Defaults:
 * - `group_name`: `"fr3_arm"`.
 * - `link_name`: `"fr3_hand_tcp"` (used only for pose goals).
 */
class BuildMotionPlanRequest final : public BT::SyncActionNode
{
public:
  BuildMotionPlanRequest(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      // Optional (defaults are FR3-specific)
      BT::InputPort<std::string>(PortKeys::GROUP_NAME, "fr3_arm", ""),

      BT::InputPort<geometry_msgs::msg::PoseStamped>(PortKeys::TARGET_POSE),
      BT::InputPort<std::string>(PortKeys::LINK_NAME, "fr3_hand_tcp", ""),

      // Joint goal (useful for direct joint targets)
      BT::InputPort<std::vector<std::string>>(PortKeys::JOINT_NAMES),
      BT::InputPort<std::vector<double>>(PortKeys::JOINT_POSITIONS),

      // Tolerances (used for whichever goal form is used)
      BT::InputPort<double>(PortKeys::POSITION_TOLERANCE, 0.005, ""),
      BT::InputPort<double>(PortKeys::ORIENTATION_TOLERANCE, 0.05, ""),
      BT::InputPort<double>(PortKeys::JOINT_TOLERANCE, 0.001, ""),

      // Scaling factors
      BT::InputPort<double>(PortKeys::MAX_VELOCITY_SCALING_FACTOR, 1.0, ""),
      BT::InputPort<double>(PortKeys::MAX_ACCELERATION_SCALING_FACTOR, 1.0, ""),

      // Output
      BT::OutputPort<std::shared_ptr<const moveit_msgs::msg::MotionPlanRequest>>(
          PortKeys::REQUEST),

      // Optional: debugging string
      BT::OutputPort<std::string>(PortKeys::ERROR),
      BT::OutputPort<std::string>(PortKeys::FEEDBACK),
      BT::OutputPort<double>(PortKeys::ELAPSED_SEC),
    };
  }

  /**
   * @brief Names of BT ports used by BuildMotionPlanRequest.
   */
  struct PortKeys: bt_utils::DiagnosticPortKeys
  {
    static inline const std::string GROUP_NAME = "group_name";
    static inline const std::string TARGET_POSE = "target_pose";
    static inline const std::string LINK_NAME = "link_name";
    static inline const std::string JOINT_NAMES = "joint_names";
    static inline const std::string JOINT_POSITIONS = "joint_positions";
    static inline const std::string POSITION_TOLERANCE = "position_tolerance";
    static inline const std::string ORIENTATION_TOLERANCE = "orientation_tolerance";
    static inline const std::string JOINT_TOLERANCE = "joint_tolerance";
    static inline const std::string MAX_VELOCITY_SCALING_FACTOR =
        "max_velocity_scaling_factor";
    static inline const std::string MAX_ACCELERATION_SCALING_FACTOR =
        "max_acceleration_scaling_factor";
    static inline const std::string REQUEST = "request";
  };

  BT::NodeStatus tick() override;

private:
  BT::NodeStatus failWithError(const std::string& msg);

  std::shared_ptr<RuntimeContext> ctx_;

  std::optional<rclcpp::Logger> logger_;
};

}  // namespace rlc_executive
