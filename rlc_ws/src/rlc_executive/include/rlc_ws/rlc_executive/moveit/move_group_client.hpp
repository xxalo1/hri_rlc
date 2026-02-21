#pragma once

#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>

#include "rlc_executive/core/exec_config.hpp"
#include "rlc_executive/core/types.hpp"

namespace rlc_executive
{

class MoveGroupClient final
{
public:
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using Client = rclcpp_action::Client<MoveGroup>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<MoveGroup>;

  explicit MoveGroupClient(rclcpp::Node::SharedPtr& node, const ExecConfig& cfg);

  PlanResult plan(const moveit_msgs::msg::MotionPlanRequest& req,
                  const PlanningProfile& profile);

private:
  rclcpp::Node::SharedPtr node_ = nullptr;     // non-owning
  const ExecConfig* cfg_ = nullptr;  // non-owning

  Client::SharedPtr client_;
};

}  // namespace rlc_executive