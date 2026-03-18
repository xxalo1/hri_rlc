#pragma once

#include <atomic>
#include <string>

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "rlc_executive/core/exec_config.hpp"

#include "rlc_executive/moveit/teleoperation_session.hpp"

namespace rlc_executive
{

class ServoJoyFrontend final
{
public:
  enum class CommandMode
  {
    TWIST,
    JOINT_JOG,
    POSE
  };

  explicit ServoJoyFrontend(rclcpp::Node& node, const ServoJoyFrontendConfig& cfg,
                            TeleoperationSession& teleop_session);

  bool isEnabled() const noexcept
  { return enabled_.load(std::memory_order_relaxed); }

  void setEnabled(bool enabled) noexcept
  { enabled_.store(enabled, std::memory_order_relaxed); }

  CommandMode commandMode() const noexcept
  { return mode_.load(std::memory_order_relaxed); }

  void setCommandMode(CommandMode mode) noexcept
  { mode_.store(mode, std::memory_order_relaxed); }

private:
  void onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr& msg);

  void publishServoCommand(const sensor_msgs::msg::Joy& joy);

  geometry_msgs::msg::TwistStamped
  makeTwistCommand(const sensor_msgs::msg::Joy& joy) const;
  control_msgs::msg::JointJog makeJointJogCommand(const sensor_msgs::msg::Joy& joy) const;
  geometry_msgs::msg::PoseStamped makePoseCommand(const sensor_msgs::msg::Joy& joy) const;

  double axisValue(const sensor_msgs::msg::Joy& joy, int index) const noexcept;
  double triggerAxisValue(const sensor_msgs::msg::Joy& joy, int pos_index,
                          int neg_index) const noexcept;

  double buttonPairValue(const sensor_msgs::msg::Joy& joy, int pos_index,
                         int neg_index) const noexcept;
  bool isButtonPressed(const sensor_msgs::msg::Joy& joy, int index) const noexcept;

  std::optional<TeleoperationSession::DemoRequest>
  detectDemoRequest(const sensor_msgs::msg::Joy& joy);

private:
  rclcpp::Node* node_ = nullptr;
  ServoJoyFrontendConfig cfg_;
  TeleoperationSession& teleop_session_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  std::atomic_bool enabled_{ false };
  std::atomic<CommandMode> mode_{ CommandMode::TWIST };

  std::string command_frame_;
  bool finish_button_prev_ = false;
  bool abort_button_prev_ = false;
};

}  // namespace rlc_executive
