#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rlc_common/endpoints.hpp"

#include "rbt_core_cpp/robot.hpp"
#include "rbt_core_cpp/types.hpp"

#include "rlc_utils/types.hpp"

namespace rlc_controller_cpp
{

class ControllerNode final : public rclcpp::Node
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  using JointStateMsg = rlc_common::JointStateMsg;
  using JointEffortCmdMsg = rlc_common::JointEffortCmdMsg;
  using ControllerStateMsg = rlc_common::ControllerStateMsg;
  using JointStateMsgData = rlc_utils::types::JointStateMsgData;
  using JointEffortCmdMsgData = rlc_utils::types::JointEffortCmdMsgData;
  using JointControllerStateMsgData = rlc_utils::types::JointControllerStateMsgData;
  using Vec = rbt_core_cpp::Vec;

  struct Params
  {
    double controller_rate_hz{1000.0};           // target
  };

private:
  // ROS callbacks
  void joint_state_cb(const JointStateMsg::SharedPtr msg);
  void control_step();

  // One-time configuration on first JointState (control thread only).
  void configure_from_first_state(const JointStateMsg& msg);

  // Hot-path helper (no allocations)
  void fill_effort_cmd_msg(const Vec& tau_src, const rclcpp::Time& stamp);

  Eigen::Index n() const noexcept { return joint_state_.size(); }

private:
  Params params_{};

  // Callback groups: state subscription + control timer may run concurrently under MultiThreadedExecutor.
  rclcpp::CallbackGroup::SharedPtr cb_ctrl_;
  rclcpp::CallbackGroup::SharedPtr cb_state_;


  rclcpp::Subscription<JointStateMsg>::SharedPtr sub_js_;
  rclcpp::Publisher<JointEffortCmdMsg>::SharedPtr pub_effort_;
  rclcpp::TimerBase::SharedPtr timer_;

  rbt_core_cpp::Robot robot_;

  // Latest JointState mailbox (subscription thread -> control thread).
  std::shared_ptr<const JointStateMsg> latest_js_msg_;
  std::atomic<std::uint64_t> latest_js_seq_{0};
  std::uint64_t consumed_js_seq_{0};  // control thread only

  JointStateMsgData joint_state_;

  JointControllerStateMsgData ctrl_state_;
  JointEffortCmdMsgData joint_cmd_;
  JointEffortCmdMsg joint_cmd_msg_;
  ControllerStateMsg ctrl_state_msg_;

  std::atomic<bool> io_configured_{false};

  


};

}  // namespace rlc_controller_cpp
