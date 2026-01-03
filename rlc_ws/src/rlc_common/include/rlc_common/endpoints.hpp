#pragma once

// ROS interfaces
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

// Your interfaces (match your Python imports)
#include <rlc_interfaces/msg/current_plan.hpp>
#include <rlc_interfaces/msg/joint_effort_cmd.hpp>
#include <rlc_interfaces/msg/joint_state_action.hpp>
#include <rlc_interfaces/msg/planned_cartesian_trajectory.hpp>
#include <rlc_interfaces/msg/planned_joint_trajectory.hpp>

#include <rlc_interfaces/srv/execute_trajectory.hpp>
#include <rlc_interfaces/srv/plan_joint_trajectory.hpp>
#include <rlc_interfaces/srv/set_controller_gains.hpp>
#include <rlc_interfaces/srv/set_controller_mode.hpp>

namespace rlc_common
{

template <class T>
struct Endpoint
{
  const char * name;
  using type = T;
};

// Use macros for namespaces so we can do compile time string literal concatenation:
//   SIM_NS "/joint_state"  -> "sim/gen3/joint_state"
#define SIM_NS  "sim/gen3"
#define CTRL_NS "ctrl/gen3"
#define PLAN_NS "plan/gen3"
#define EXEC_NS "exec/gen3"
#define MON_NS  "mon/gen3"

struct TopicEndpoints
{
  Endpoint<sensor_msgs::msg::JointState> joint_state{SIM_NS "/joint_state"};
  Endpoint<rlc_interfaces::msg::JointStateAction> joint_state_action{SIM_NS "/joint_state_action"};
  Endpoint<rlc_interfaces::msg::JointEffortCmd> effort_cmd{CTRL_NS "/effort_cmd"};
  Endpoint<rlc_interfaces::msg::PlannedJointTrajectory> planned_joint_traj{PLAN_NS "/planned_joint_traj"};
  Endpoint<geometry_msgs::msg::PoseArray> frame_states{MON_NS "/frame_states"};
  Endpoint<rlc_interfaces::msg::PlannedCartesianTrajectory> planned_cart_traj{PLAN_NS "/planned_cart_traj"};
  Endpoint<control_msgs::msg::JointTrajectoryControllerState> controller_state{CTRL_NS "/controller_state"};
  Endpoint<rlc_interfaces::msg::CurrentPlan> current_plan{EXEC_NS "/current_plan"};
};

struct ServiceEndpoints
{
  Endpoint<std_srvs::srv::Trigger> reset_sim{SIM_NS "/reset"};
  Endpoint<std_srvs::srv::SetBool> pause_sim{SIM_NS "/pause"};
  Endpoint<rlc_interfaces::srv::SetControllerGains> set_controller_gains{CTRL_NS "/set_controller_gains"};
  Endpoint<rlc_interfaces::srv::SetControllerMode> set_controller_mode{CTRL_NS "/set_control_mode"};
  Endpoint<rlc_interfaces::srv::PlanJointTrajectory> plan_quintic{PLAN_NS "/plan_quintic"};
  Endpoint<rlc_interfaces::srv::ExecuteTrajectory> execute_traj{EXEC_NS "/execute"};
};

struct ActionEndpoints
{
  Endpoint<control_msgs::action::FollowJointTrajectory> follow_traj{CTRL_NS "/follow_trajectory"};
};

// Global, header only, no YAML, no init functions.
inline constexpr TopicEndpoints TOPICS{};
inline constexpr ServiceEndpoints SERVICES{};
inline constexpr ActionEndpoints ACTIONS{};

// Convenience aliases (mirror your Python intent).
using JointStateMsg = typename decltype(TOPICS.joint_state)::type;
using JointEffortCmdMsg = typename decltype(TOPICS.effort_cmd)::type;
using PlannedJointTrajMsg = typename decltype(TOPICS.planned_joint_traj)::type;
using FrameStatesMsg = typename decltype(TOPICS.frame_states)::type;
using PlannedCartTrajMsg = typename decltype(TOPICS.planned_cart_traj)::type;
using ControllerStateMsg = typename decltype(TOPICS.controller_state)::type;
using CurrentPlanMsg = typename decltype(TOPICS.current_plan)::type;
using JointStateActionMsg = typename decltype(TOPICS.joint_state_action)::type;

using ResetSimSrv = typename decltype(SERVICES.reset_sim)::type;
using PauseSimSrv = typename decltype(SERVICES.pause_sim)::type;
using SetControllerGainsSrv = typename decltype(SERVICES.set_controller_gains)::type;
using SetControllerModeSrv = typename decltype(SERVICES.set_controller_mode)::type;
using PlanQuinticSrv = typename decltype(SERVICES.plan_quintic)::type;
using ExecuteTrajSrv = typename decltype(SERVICES.execute_traj)::type;

using FollowTrajAction = typename decltype(ACTIONS.follow_traj)::type;

#undef SIM_NS
#undef CTRL_NS
#undef PLAN_NS
#undef EXEC_NS
#undef MON_NS

}  // namespace rlc_common
