#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rlc_common/endpoints.hpp"

#include "rbt_core_cpp/robot.hpp"
#include "rbt_core_cpp/types.hpp"

#include "ros_utils_cpp/msg_conv.hpp"

rmsg = namespace ros_utils_cpp::msg_conv;
namespace rlc_controller_cpp
{

class ControllerNode final : public rclcpp::Node
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  using JointStateMsg = rlc_common::JointStateMsg;
  using EffortCmdMsg = rlc_common::JointEffortCmdMsg;
  using Vec = rbt_core_cpp::Vec;

  struct Params
  {
    double controller_rate_hz{1000.0};           // target
  };

  struct JointStateBuf
  {
    rclcpp::Time stamp{};
    std::vector<std::string> names;  // copied once (first msg), then treated as immutable

    Vec q;    // src order
    Vec qd;   // src order
    Vec qdd;  // src order (kept, but typically zero)

    bool has_state{false};
  };

  struct DesiredBuf
  {
    Vec q;    // src order
    Vec qd;   // src order
    Vec qdd;  // src order
  };

  struct MsgCache
  {
    EffortCmdMsg effort_cmd;
  };

private:
  // ROS callbacks
  void joint_state_cb(const JointStateMsg::SharedPtr msg);
  void control_step();

  // One-time configuration on first JointState
  void configure_from_first_state(const JointStateMsg& msg);

  // Hot-path helper (no allocations)
  void fill_effort_cmd_msg(const Vec& tau_src, const rclcpp::Time& stamp);

private:
  Params params_{};

  // Callback group to guarantee no concurrent state/timer access even under MultiThreadedExecutor
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  rclcpp::Subscription<JointStateMsg>::SharedPtr sub_js_;
  rclcpp::Publisher<EffortCmdMsg>::SharedPtr pub_effort_;
  rclcpp::TimerBase::SharedPtr timer_;

  rbt_core_cpp::Robot robot_;
  int n_{0};
  bool io_configured_{false};

  JointStateBuf state_;
  DesiredBuf des_;
  MsgCache msg_;
};

}  // namespace rlc_controller_cpp
