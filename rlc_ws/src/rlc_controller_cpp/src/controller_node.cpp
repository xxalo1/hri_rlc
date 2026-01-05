#include "rlc_controller_cpp/controller_node.hpp"

#include <Eigen/Core>
#include <chrono>
#include <stdexcept>

#include "rlc_robot_models/kinova_gen3.hpp"
#include "rlc_utils/msg_conv.hpp"
namespace rlc_controller_cpp {
namespace rumsg = rlc_utils::msg_conv;

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("gen3_controller", options),
      robot_(rlc_robot_models::make_gen3_spec()) {
  // ----------------------------
  // Parameters (decl + read)
  // ----------------------------
  this->declare_parameter<double>("controller_rate_hz",
                                  params_.controller_rate_hz);
  params_.controller_rate_hz =
      this->get_parameter("controller_rate_hz").as_double();

  if (params_.controller_rate_hz <= 0.0) {
    throw std::runtime_error("controller_rate_hz must be > 0");
  }

  cb_ctrl_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_state_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // QoS similar to "latest"
  auto qos_latest =
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  // ----------------------------
  // Publishers / Subscribers
  // ----------------------------
  constexpr auto topics = rlc_common::TOPICS;

  pub_effort_ = this->create_publisher<JointEffortCmdMsg>(
      topics.effort_cmd.name, qos_latest);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_state_;
  sub_js_ = this->create_subscription<JointStateMsg>(
      topics.joint_state.name, qos_latest,
      std::bind(&ControllerNode::joint_state_cb, this, std::placeholders::_1),
      sub_opts);

  // ----------------------------
  // Timer: control step
  // ----------------------------
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / params_.controller_rate_hz));

  timer_ = this->create_wall_timer(
      period_ns, std::bind(&ControllerNode::control_step, this), cb_ctrl_);

  RCLCPP_INFO(this->get_logger(),
              "gen3_controller_cpp started. controller_rate_hz=%.3f",
              params_.controller_rate_hz);
}

void ControllerNode::configure_from_first_state(const JointStateMsg& msg) {
  std::size_t n = msg.name.size();

  if (n == 0) {
    throw std::runtime_error(
        "Received JointState msg with zero joint names in "
        "configure_from_first_state");
  }

  if (msg.position.size() != n) {
    throw std::runtime_error(
        "JointState msg position size does not match name size");
  }

  // check robot compatibility with recieved joint length
  if (robot_.n() != static_cast<int>(n)) {
    throw std::runtime_error(
        "Robot model joint count does not match recieved JointState msg");
  }

  const Eigen::Index ni = static_cast<Eigen::Index>(n);

  // resize preallocated containers
  joint_state_.resize(ni);
  ctrl_state_.resize(ni);
  joint_cmd_.resize(ni);

  // construct outgoing messages once
  joint_cmd_msg_ = rumsg::make_joint_effort_cmd_msg(n, 0.0, msg.name);
  ctrl_state_msg_ = rumsg::make_joint_ctrl_state_msg(n, 0.0, msg.name);

  // copy joint names
  joint_state_.name = msg.name;
  ctrl_state_.joint_names = msg.name;
  ctrl_state_msg_.joint_names = msg.name;
  joint_cmd_.name = msg.name;
  joint_cmd_msg_.name = msg.name;

  // configure robot I/O
  robot_.configure_io(joint_state_.name);
}

void ControllerNode::joint_state_cb(const JointStateMsg::SharedPtr msg) {
  if (!msg) return;

  std::atomic_store_explicit(&latest_js_msg_,
                             std::static_pointer_cast<const JointStateMsg>(msg),
                             std::memory_order_release);
  latest_js_seq_.fetch_add(1, std::memory_order_release);
}

void ControllerNode::control_step() {
  const std::uint64_t seq = latest_js_seq_.load(std::memory_order_acquire);
  if (seq == consumed_js_seq_) return;
  consumed_js_seq_ = seq;

  const auto msg =
      std::atomic_load_explicit(&latest_js_msg_, std::memory_order_acquire);
  if (!msg) return;

  if (!io_configured_.load(std::memory_order_acquire)) {
    configure_from_first_state(*msg);
    io_configured_.store(true, std::memory_order_release);
    return;
  }

  if (!rumsg::from_joint_state_msg(*msg, joint_state_)) return;

  robot_.set_joint_state(&joint_state_.position, &joint_state_.velocity,
                         nullptr, &joint_state_.stamp_sec);

  joint_cmd_.effort.noalias() = robot_.compute_ctrl_effort();
  joint_cmd_.stamp_sec = joint_state_.stamp_sec;
  rumsg::to_joint_effort_cmd_msg(joint_cmd_, joint_cmd_msg_);

  pub_effort_->publish(joint_cmd_msg_);
}

}  // namespace rlc_controller_cpp
