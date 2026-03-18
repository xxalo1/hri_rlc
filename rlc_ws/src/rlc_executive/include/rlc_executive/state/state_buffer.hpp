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
  /**
   * @brief Subscribes to robot state topics and caches the latest snapshots.
   * @param[in] node ROS node used to create subscriptions, timers, and TF utilities.
   * @param[in] cfg State configuration with topic names, QoS, frame ids, and timeouts.
   * @throws std::invalid_argument If required topic, frame, or timeout settings are invalid.
   */
  StateBuffer(rclcpp::Node& node, const StateConfig& cfg);

  /**
   * @brief Reports whether at least one joint-state message has been cached.
   * @return `true` if a joint-state snapshot is available, otherwise `false`.
   */
  bool hasJointState() const;

  /**
   * @brief Returns the latest cached joint-state message without deep-copying it.
   * @return Shared pointer to the latest joint-state message; may be null if no
   * joint-state message has been received yet.
   */
  sensor_msgs::msg::JointState::ConstSharedPtr getLatestJointState() const;

  /**
   * @brief Reports whether the cached joint-state timestamp is still fresh.
   * @return `true` if the latest joint-state message is available and its age is at
   * most `cfg.state_stale_sec`, otherwise `false`.
   */
  bool isJointStateFresh() const;

  /**
   * @brief Looks up the configured base-to-end-effector transform.
   * @return Latest transform from `cfg.base_frame` to `cfg.ee_frame`, or
   * `std::nullopt` if the lookup fails within the configured timeout.
   */
  std::optional<geometry_msgs::msg::TransformStamped> tryLookupBaseToEe() const;

  /**
   * @brief Looks up a transform between two TF frames.
   * @param[in] target_frame TF target frame name.
   * @param[in] source_frame TF source frame name.
   * @return Latest transform from `source_frame` to `target_frame`, or
   * `std::nullopt` if the lookup fails within the configured timeout.
   */
  std::optional<geometry_msgs::msg::TransformStamped>
  tryLookupTransform(const std::string& target_frame,
                     const std::string& source_frame) const;

private:
  void onJointState(const sensor_msgs::msg::JointState::ConstSharedPtr& msg);

  rclcpp::Node* node_ = nullptr;     // non-owning
  const StateConfig* cfg_ = nullptr;  // non-owning

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  mutable std::mutex mtx_;
  sensor_msgs::msg::JointState::ConstSharedPtr latest_joint_state_;
};

}  // namespace rlc_executive
