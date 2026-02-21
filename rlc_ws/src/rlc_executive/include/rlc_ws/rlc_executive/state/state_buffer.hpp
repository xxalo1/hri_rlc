#pragma once

#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rlc_executive/core/exec_config.hpp"

namespace rlc_executive
{

class StateBuffer final
{
public:
  StateBuffer(rclcpp::Node& node, const ExecConfig& cfg);

  bool hasJointState() const;

  std::optional<sensor_msgs::msg::JointState> getLatestJointState() const;

  bool isJointStateFresh() const;

  std::optional<geometry_msgs::msg::TransformStamped> tryLookupBaseToEe() const;

  std::optional<geometry_msgs::msg::TransformStamped>
  tryLookupTransform(const std::string& target_frame,
                     const std::string& source_frame) const;

private:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Node* node_ = nullptr;     // non-owning
  const ExecConfig* cfg_ = nullptr;  // non-owning

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  mutable std::mutex mtx_;
  std::optional<sensor_msgs::msg::JointState> latest_joint_state_;
};

}  // namespace rlc_executive