#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>

#include "rlc_executive/core/exec_config.hpp"
#include "rlc_executive/core/types.hpp"

namespace rlc_executive
{

struct PlanningFeedback
{
  std::string text;
  double elapsed_sec = 0.0;
};

class MoveGroupClient final
{
public:
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using Client = rclcpp_action::Client<MoveGroup>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<MoveGroup>;

  MoveGroupClient(rclcpp::Node& node, const ExecConfig& cfg);

  // Non-blocking: send plan request and return immediately.
  bool start(const moveit_msgs::msg::MotionPlanRequest& req,
             const PlanningProfile& profile, std::string* error_msg = nullptr);

  // Best-effort cancel any active planning goal, then start new one.
  bool preemptAndStart(const moveit_msgs::msg::MotionPlanRequest& req,
                       const PlanningProfile& profile, std::string* error_msg = nullptr);

  // Best-effort cancel request (non-blocking).
  void requestCancel();

  bool isActive() const;
  bool hasResult() const;

  std::optional<PlanResult> peekResult() const;
  std::optional<PlanResult> takeResult();

  PlanningFeedback latestFeedback() const;

private:
  void handleGoalResponse(std::uint64_t seq, const GoalHandle::SharedPtr& goal_handle);
  void handleFeedback(std::uint64_t seq);
  void handleResult(std::uint64_t seq, const GoalHandle::WrappedResult& wrapped);

  rclcpp::Node* node_ = nullptr;
  const ExecConfig* cfg_ = nullptr;

  Client::SharedPtr client_;

  mutable std::mutex mtx_;

  std::uint64_t active_seq_ = 0;
  bool active_ = false;

  GoalHandle::SharedPtr active_goal_;

  rclcpp::Time start_time_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time last_feedback_time_{ 0, 0, RCL_ROS_TIME };
  std::string last_feedback_text_;

  std::optional<PlanResult> result_;
};

}  // namespace rlc_executive
