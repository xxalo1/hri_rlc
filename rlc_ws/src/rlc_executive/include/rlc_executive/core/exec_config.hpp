#pragma once

#include <map>
#include <string>

#include <rclcpp/qos.hpp>

#include "rlc_executive/core/types.hpp"
#include <chrono>

namespace rlc_executive
{

enum class TickOption : std::uint8_t
{
  ONCE_UNLESS_WOKEN_UP = 0,
  WHILE_RUNNING = 1,
  EXACTLY_ONCE = 2
};

struct ExecConfig
{
  double tick_rate_hz = 30.0;

  std::string default_tree_xml;

  std::string joint_states_topic = "/joint_states";
  std::string base_frame = "base_link";
  std::string ee_frame = "tool0";

  double tf_timeout_sec = 0.05;
  double state_stale_sec = 0.2;

  std::string move_group_action_name = "/move_action";
  std::string execute_traj_action_name = "/execute_trajectory";
  std::string get_planning_scene_service_name = "/get_planning_scene";

  std::string default_planning_profile;
  std::map<std::string, PlanningProfile> planning_profiles;

  rclcpp::QoS joint_state_qos = rclcpp::SensorDataQoS();

  TickOption tick_option = TickOption::ONCE_UNLESS_WOKEN_UP;
  std::chrono::milliseconds tick_while_running_max_block{ 100 };
};

}  // namespace rlc_executive