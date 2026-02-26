#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace rlc_executive
{

namespace consts
{
inline constexpr const char* EXECUTOR_MODE_MOVEIT = "moveit";
inline constexpr const char* EXECUTOR_MODE_DIRECT_FJT = "direct_fjt";
}  // namespace consts

enum class ExecStatus : std::uint8_t
{
  UNKNOWN = 0,
  SUCCESS = 1,
  FAILURE = 2,
  CANCELED = 3,
  TIMEOUT = 4
};

enum class PlanStatus : std::uint8_t
{
  UNKNOWN = 0,
  SUCCESS = 1,
  FAILURE = 2,
  TIMEOUT = 3
};

struct PlanningProfile
{
  std::string name;

  // MoveIt request selectors
  std::string pipeline_id;  // e.g. "ompl" or "rlc_trajopt"
  std::string planner_id;   // e.g. "ompl" or "rlc_trajopt"

  // Common request knobs
  double allowed_planning_time_sec = 5.0;
  int num_planning_attempts = 1;

  // Execution backend preference: consts::EXECUTOR_MODE_*
  std::string executor_mode = consts::EXECUTOR_MODE_MOVEIT;

  // Only used if executor_mode == consts::EXECUTOR_MODE_DIRECT_FJT
  std::string fjt_action_name;  // e.g. "/arm_controller/follow_joint_trajectory"
};

struct TaskContext
{
  std::string task_id;  // e.g. "reach_pose", "hold", "follow_path" (generic)
  std::string tree_id;  // which BT XML to load; optional override

  // Optional high-level goal (keep generic; you can expand later)
  std::optional<geometry_msgs::msg::PoseStamped> target_pose;

  // Profile selection (maps to PlanningProfile loaded from YAML)
  std::string planning_profile;

  // Extension point for research metadata, without schema churn
  std::map<std::string, std::string> metadata;
};

struct StateSnapshot
{
  builtin_interfaces::msg::Time stamp;

  sensor_msgs::msg::JointState joint_state;

  // Optional, only set if you choose to cache it
  std::optional<geometry_msgs::msg::PoseStamped> ee_pose;

  // Freshness flags set by state acquisition nodes
  bool joint_state_valid = false;
  bool tf_valid = false;
};

/**
 * @brief Summary of a motion planning attempt for BT outputs.
 *
 * @details
 * This type intentionally excludes the planned trajectory to keep BehaviorTree
 * blackboard values small. Publish the trajectory separately (e.g. via a BT output
 * port of type `std::shared_ptr<const moveit_msgs::msg::RobotTrajectory>`).
 *
 * Fields:
 * - `status`: Planning status.
 * - `moveit_error_code`: MoveIt error code (`moveit_msgs::msg::MoveItErrorCodes::val`).
 * - `message`: Human-readable message.
 * - `planning_time_sec`: Planning time [s].
 */
struct PlanSummary
{
  PlanStatus status = PlanStatus::UNKNOWN;

  int32_t moveit_error_code = 0;
  std::string message;

  double planning_time_sec = 0.0;
};

/**
 * @brief Result of a MoveIt planning attempt (internal).
 *
 * @details
 * This type contains the full planned trajectory by value for convenience inside
 * planning clients. BT nodes should prefer publishing a lightweight summary
 * (`PlanSummary`) and publishing the trajectory separately via a shared pointer.
 */
struct PlanResult
{
  PlanStatus status = PlanStatus::UNKNOWN;

  moveit_msgs::msg::RobotTrajectory trajectory;

  // Error details for debugging
  int32_t moveit_error_code = 0;
  std::string message;

  double planning_time_sec = 0.0;
};

struct ExecResult
{
  ExecStatus status = ExecStatus::UNKNOWN;

  int32_t error_code = 0;
  std::string message;

  double execution_time_sec = 0.0;
};

struct PlanningRequest
{
  moveit_msgs::msg::MotionPlanRequest moveit_request;
  PlanningProfile profile;
};

}  // namespace rlc_executive
