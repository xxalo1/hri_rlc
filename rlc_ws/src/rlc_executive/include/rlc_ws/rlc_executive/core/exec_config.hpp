#pragma once

#include <chrono>
#include <map>
#include <string>

#include <rclcpp/qos.hpp>

#include "rlc_executive/core/types.hpp"

namespace rlc_executive
{

struct ExecConfig
{
  // BT ticking
  double tick_rate_hz = 30.0;

  // Default BT tree file name under share/<pkg>/bt_trees/
  // Example: "reactive_core.xml"
  std::string default_tree_xml;

  // ROS topics and frames
  std::string joint_states_topic = "/joint_states";
  std::string base_frame = "base_link";
  std::string ee_frame = "tool0";

  // TF lookup behavior
  double tf_timeout_sec = 0.05;
  double state_stale_sec = 0.2;

  // MoveIt endpoints
  std::string move_group_action_name = "/move_action";
  std::string execute_traj_action_name = "/execute_trajectory";
  std::string get_planning_scene_service_name = "/get_planning_scene";

  // Planning profiles loaded from YAML
  std::string default_planning_profile;
  std::map<std::string, PlanningProfile> planning_profiles;

  // QoS (keep simple; can be expanded later)
  rclcpp::QoS joint_state_qos = rclcpp::SensorDataQoS();
};

}  // namespace rlc_executive_bt