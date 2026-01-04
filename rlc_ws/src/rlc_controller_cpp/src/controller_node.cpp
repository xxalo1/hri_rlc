#include "rlc_controller_cpp/controller_node.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <stdexcept>

#include "rlc_robot_models/kinova_gen3.hpp"
namespace rlc_controller_cpp {
using namespace std::chrono_literals;

static inline double to_sec(const rclcpp::Time& t) { return t.seconds(); }

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

  // After construction:
  n_ = robot_.n();

  // Preallocate buffers (once n_ is known)
  state_.q.resize(n_);
  state_.qd.resize(n_);
  state_.qdd.resize(n_);
  des_.q.resize(n_);
  des_.qd.resize(n_);
  des_.qdd.resize(n_);
  state_.qdd.setZero();
  des_.qd.setZero();
  des_.qdd.setZero();

  // Set base pose
  // Eigen::Matrix<double,7,1> pose;
  // for (int i = 0; i < 7; ++i) pose[i] =
  // params_.base_pose_wb[static_cast<size_t>(i)];
  // robot_->set_base_pose_wb(pose);

  // Set defaults (adjust to your controller API)
  // robot_->set_ctrl_mode(rbt_core_cpp::CtrlMode::CT);
  // robot_->ctrl().set_joint_gains(/*kp=*/1.0, /*kv=*/1.0, /*ki=*/1.0);

  // ----------------------------
  // Preallocate buffers (once n_ is known)
  // ----------------------------
  // state_.q.resize(n_);  state_.qd.resize(n_);  state_.qdd.resize(n_);
  // des_.q.resize(n_);    des_.qd.resize(n_);    des_.qdd.resize(n_);
  // state_.qdd.setZero(); des_.qd.setZero(); des_.qdd.setZero();

  // ----------------------------
  // Callback group: mutually exclusive (no locks, no races)
  // ----------------------------
  cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // QoS similar to "latest"
  auto qos_latest =
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  // ----------------------------
  // Publishers / Subscribers
  // ----------------------------
  constexpr auto topics = rlc_common::TOPICS;

  pub_effort_ =
      this->create_publisher<EffortCmdMsg>(topics.effort_cmd.name, qos_latest);

  sub_js_ = this->create_subscription<JointStateMsg>(
      topics.joint_state.name, qos_latest,
      std::bind(&ControllerNode::joint_state_cb, this, std::placeholders::_1),
      rclcpp::SubscriptionOptions{.callback_group = cb_group_});

  // ----------------------------
  // Timer: control step
  // ----------------------------
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / params_.controller_rate_hz));

  timer_ = this->create_wall_timer(
      period_ns, std::bind(&ControllerNode::control_step, this), cb_group_);

  this->get_logger().info(("gen3_controller_cpp started. controller_rate_hz=" +
                           std::to_string(params_.controller_rate_hz))
                              .c_str());
}

void ControllerNode::configure_from_first_state(const JointStateMsg& msg) {
  // Copy names once (string allocations happen once here, not in hot path)
  state_.names = msg.name;

  if (static_cast<int>(state_.names.size()) != n_) {
    throw std::runtime_error("JointState name size != robot DOF");
  }
  if (static_cast<int>(msg.position.size()) != n_) {
    throw std::runtime_error("JointState position size != robot DOF");
  }
  if (!msg.velocity.empty() && static_cast<int>(msg.velocity.size()) != n_) {
    throw std::runtime_error("JointState velocity size != robot DOF");
  }

  // Configure mapping once.
  // If you need a prefix later, call robot_.configure_io(state_.names,
  // "gen3_").
  robot_.configure_io(state_.names);

  // Pre-size outgoing message once (no resizing later)
  msg_.effort_cmd.name = state_.names;
  msg_.effort_cmd.effort.resize(static_cast<size_t>(n_));  // allocate once

  io_configured_ = true;
}

void ControllerNode::joint_state_cb(const JointStateMsg::SharedPtr msg) {
  if (!msg) return;

  // One-time config on first message
  if (!io_configured_) {
    configure_from_first_state(*msg);
  }

  // Stamp
  state_.stamp = msg->header.stamp;

  // Copy into preallocated Eigen vectors (no allocations)
  // Positions are required
  {
    const double* p = msg->position.data();
    for (int i = 0; i < n_; ++i) state_.q[i] = p[i];
  }

  // Velocities optional
  if (!msg->velocity.empty()) {
    const double* v = msg->velocity.data();
    for (int i = 0; i < n_; ++i) state_.qd[i] = v[i];
  } else {
    state_.qd.setZero();
  }

  // Accelerations not provided in JointState; keep zero
  state_.qdd.setZero();

  state_.has_state = true;
}

void ControllerNode::fill_effort_cmd_msg(const Vec& tau_src,
                                         const rclcpp::Time& stamp) {
  // tau_src is src order already per Robot::compute_ctrl_effort() contract
  msg_.effort_cmd.header.stamp = stamp;

  auto& e = msg_.effort_cmd.effort;
  for (int i = 0; i < n_; ++i) {
    e[static_cast<size_t>(i)] = tau_src[i];
  }
}

void ControllerNode::control_step() {
  if (!io_configured_) return;
  if (!state_.has_state) return;

  // Update robot state (src order in)
  const double t_sec = to_sec(state_.stamp);
  robot_.set_joint_state(&state_.q, &state_.qd, &state_.qdd, &t_sec);

  // Desired policy for phase 1: HOLD current q, zero qd/qdd
  // (no trajectory logic yet)
  des_.q = state_.q;  // O(n) copy, no alloc
  des_.qd.setZero();
  des_.qdd.setZero();
  robot_.set_joint_des(&des_.q, &des_.qd, &des_.qdd);

  // Compute effort (no allocations expected inside if Robot is implemented
  // correctly)
  const Vec& tau_src = robot_.compute_ctrl_effort();

  // Publish
  fill_effort_cmd_msg(tau_src, state_.stamp);
  pub_effort_->publish(msg_.effort_cmd);
}

}  // namespace rlc_controller_cpp
