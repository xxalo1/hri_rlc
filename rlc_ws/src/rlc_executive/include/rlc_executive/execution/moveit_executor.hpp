#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <cstdint>

#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "rlc_executive/core/exec_config.hpp"
#include "rlc_executive/core/types.hpp"
#include "rlc_executive/execution/trajectory_executor.hpp"

namespace rlc_executive
{

class MoveItExecutor final : public TrajectoryExecutor
{
public:
  using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
  using Client = rclcpp_action::Client<ExecuteTrajectory>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

  MoveItExecutor(rclcpp::Node& node, const ExecConfig& cfg);

  bool start(const moveit_msgs::msg::RobotTrajectory& traj,
             const PlanningProfile& profile, std::string* error_msg = nullptr) override;

  bool preemptAndStart(const moveit_msgs::msg::RobotTrajectory& traj,
                       const PlanningProfile& profile,
                       std::string* error_msg = nullptr) override;

  void requestCancel() override;

  bool isActive() const override;
  bool hasResult() const override;

  std::optional<ExecResult> peekResult() const override;
  std::optional<ExecResult> takeResult() override;

  ExecutionFeedback latestFeedback() const override;

private:
  void setImmediateFailure(ExecResult out);

  void handleGoalResponse(std::uint64_t seq, const GoalHandle::SharedPtr& gh);
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

  std::optional<ExecResult> result_;
};

}  // namespace rlc_executive
