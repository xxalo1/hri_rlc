#include "rlc_executive/state/state_buffer.hpp"

#include <chrono>

#include <rclcpp/qos.hpp>
#include <tf2/exceptions.h>

namespace rlc_executive
{

StateBuffer::StateBuffer(rclcpp::Node& node, const ExecConfig& cfg)
  : node_(&node), cfg_(&cfg), tf_buffer_(node.get_clock()), tf_listener_(tf_buffer_)
{
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      cfg_->joint_states_topic, cfg_->joint_state_qos,
      std::bind(&StateBuffer::onJointState, this, std::placeholders::_1));
}

void StateBuffer::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  latest_joint_state_ = *msg;
}

bool StateBuffer::hasJointState() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return latest_joint_state_.has_value();
}

std::optional<sensor_msgs::msg::JointState> StateBuffer::getLatestJointState() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return latest_joint_state_;
}

bool StateBuffer::isJointStateFresh() const
{
  std::optional<sensor_msgs::msg::JointState> js;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    js = latest_joint_state_;
  }

  if (!js)
  {
    return false;
  }

  // Convert message stamp to rclcpp::Time
  const rclcpp::Time stamp(js->header.stamp);
  const rclcpp::Time now = node_->get_clock()->now();

  // If stamp is zero, we cannot evaluate freshness reliably.
  if (stamp.nanoseconds() == 0)
  {
    return true;
  }

  const double age_sec = (now - stamp).seconds();
  return age_sec <= cfg_->state_stale_sec;
}

std::optional<geometry_msgs::msg::TransformStamped>
StateBuffer::tryLookupTransform(const std::string& target_frame,
                                const std::string& source_frame) const
{
  try
  {
    const auto timeout = tf2::durationFromSec(cfg_->tf_timeout_sec);
    auto tf = tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero,
                                         timeout);
    return tf;
  }
  catch (const tf2::TransformException&)
  {
    return std::nullopt;
  }
}

std::optional<geometry_msgs::msg::TransformStamped> StateBuffer::tryLookupBaseToEe() const
{
  return tryLookupTransform(cfg_->base_frame, cfg_->ee_frame);
}

}  // namespace rlc_executive