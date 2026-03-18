#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <string>

#include <rclcpp/qos.hpp>

#include "rlc_executive/core/types.hpp"

namespace rlc_executive
{

enum class TickOption : std::uint8_t
{
  ONCE_UNLESS_WOKEN_UP = 0,
  WHILE_RUNNING = 1,
  EXACTLY_ONCE = 2
};

/**
 * @brief Configures the tree tick loop driven by `BtExecutorNode`.
 *
 * @details
 * - `rate_hz`: Target BehaviorTree tick frequency [Hz].
 * - `option`: Tick policy forwarded to `TreeRunner`.
 * - `while_running_max_block`: Maximum blocking time [ms] for `WHILE_RUNNING`.
 */
struct TickConfig
{
  double rate_hz = 30.0;
  TickOption option = TickOption::ONCE_UNLESS_WOKEN_UP;
  std::chrono::milliseconds while_running_max_block{ 100 };
};

/**
 * @brief Configures robot-state subscriptions and TF lookups.
 *
 * @details
 * - `joint_states_topic`: Source topic carrying `sensor_msgs::msg::JointState`.
 * - `base_frame`: Base frame used for end-effector transform lookups.
 * - `ee_frame`: End-effector frame used for transform lookups.
 * - `tf_timeout_sec`: TF lookup timeout [s].
 * - `state_stale_sec`: Maximum accepted joint-state age [s].
 * - `joint_state_qos`: QoS profile used for the joint-state subscription.
 */
struct StateConfig
{
  std::string joint_states_topic = "/joint_states";
  std::string base_frame = "base_link";
  std::string ee_frame = "tool0";
  double tf_timeout_sec = 0.05;
  double state_stale_sec = 0.2;
  rclcpp::QoS joint_state_qos = rclcpp::SensorDataQoS();
};

/**
 * @brief Configures MoveGroup planning endpoints.
 *
 * @details
 * - `action_name`: MoveGroup action name used by `MoveGroupClient`.
 * - `get_planning_scene_service_name`: Planning-scene service name reserved for future use.
 */
struct MoveGroupConfig
{
  std::string action_name = "/move_action";
  std::string get_planning_scene_service_name = "/get_planning_scene";
};

/**
 * @brief Configures trajectory execution endpoints.
 *
 * @details
 * - `execute_traj_action_name`: ExecuteTrajectory action name used by `MoveItExecutor`.
 */
struct ExecutionConfig
{
  std::string execute_traj_action_name = "/execute_trajectory";
};

/**
 * @brief Configures planning-profile selection and storage.
 *
 * @details
 * - `default_profile`: Default profile name resolved by `RuntimeContext`.
 * - `profiles`: Planning profiles loaded from `planning_profiles.yaml`.
 */
struct PlanningConfig
{
  std::string default_profile;
  std::map<std::string, PlanningProfile> profiles;
};

/**
 * @brief Configures the MoveIt Servo service and status endpoints.
 *
 * @details
 * - `pause_service_name`: Service name used to pause or resume Servo.
 * - `switch_command_type_service_name`: Service name used to switch Servo command mode.
 * - `status_topic`: Topic that publishes `moveit_msgs::msg::ServoStatus`.
 */
struct ServoConfig
{
  std::string pause_service_name = "/servo_node/pause_servo";
  std::string switch_command_type_service_name = "/servo_node/switch_command_type";
  std::string status_topic = "/servo_node/status";
};

/**
 * @brief Configures the joystick frontend that publishes Servo command topics.
 *
 * @details
 * - `joy_topic`: Source joystick topic carrying `sensor_msgs::msg::Joy`.
 * - `twist_topic`: Servo twist-command topic receiving `geometry_msgs::msg::TwistStamped`.
 * - `joint_topic`: Servo joint-jog topic receiving `control_msgs::msg::JointJog`.
 * - `pose_topic`: Servo pose-command topic receiving `geometry_msgs::msg::PoseStamped`.
 * - `command_frame`: Frame id attached to outgoing commands.
 */

/**
 * @brief Maps logical Cartesian directions to SDL game-controller indices.
 *
 * @details
 * - `x`: Index for the X direction.
 * - `y`: Index for the Y direction.
 * - `z_pos`: Index for the positive Z direction.
 * - `z_neg`: Index for the negative Z direction.
 */
struct ServoJoyAxisConfig
{
  int x = 0;
  int y = 0;
  int z_pos = 0;
  int z_neg = 0;
};

/**
 * @brief Maps demo-session actions to SDL game-controller button indices.
 *
 * @details
 * - `finish`: Button that requests `TeleoperationSession::DemoRequest::FINISH`.
 * - `abort`: Button that requests `TeleoperationSession::DemoRequest::ABORT`.
 */
struct ServoJoyButtonsConfig
{
  int finish = 6;  // START
  int abort = 4;   // BACK/SELECT
};

struct ServoJoyFrontendConfig
{
  std::string joy_topic = "/joy";
  std::string twist_topic = "/servo_node/delta_twist_cmds";
  std::string joint_topic = "/servo_node/delta_joint_cmds";
  std::string pose_topic = "/servo_node/pose_target_cmds";
  std::string command_frame;

  double deadzone = 0.10;
  double linear_scale = 0.25;
  double angular_scale = 0.75;

  ServoJoyAxisConfig axis_linear{ 1, 0, 5, 4 };   // LEFTY, LEFTX, TRIGGERRIGHT, TRIGGERLEFT
  ServoJoyAxisConfig axis_angular{ 3, 2, 10, 9 };  // RIGHTY, RIGHTX, RIGHTSHOULDER, LEFTSHOULDER
  ServoJoyButtonsConfig buttons;
};

/**
 * @brief Configures the executive's environment-monitor connection.
 *
 * @details
 * - `monitor_namespace`: ROS namespace used to discover the monitor endpoints.
 * - `env_name`: Optional explicit environment name forwarded to the monitor.
 * - `wait_timeout`: Maximum wait time [ms] for the monitor interface to become ready.
 */
struct MonitorConfig
{
  std::string monitor_namespace = "tesseract_monitor";
  std::string env_name;
  std::chrono::milliseconds wait_timeout{ 3000 };
};

/**
 * @brief Root runtime configuration for the executive.
 *
 * @details
 * This object owns the typed configuration loaded at startup and groups settings by
 * subsystem so each component can consume only its relevant section.
 */
struct ExecConfig
{
  TickConfig tick;
  StateConfig state;
  MoveGroupConfig move_group;
  ExecutionConfig execution;
  PlanningConfig planning;
  ServoConfig servo;
  ServoJoyFrontendConfig servo_joy_frontend;
  MonitorConfig monitor;
};

}  // namespace rlc_executive
