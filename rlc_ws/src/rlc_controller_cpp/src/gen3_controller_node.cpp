#include "rlc_controller_cpp/gen3_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <rlc_common/endpoints.hpp>
#include <rlc_robot_models/kinova_gen3.hpp>
#include <ros_utils_cpp/msg_conv.hpp>
#include <ros_utils_cpp/time.hpp>
#include <unordered_map>

namespace rlc_controller_cpp {
namespace {

rclcpp::QoS qos_latest() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

std::string to_lower_copy(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return s;
}

}  // namespace

Gen3ControllerNode::Gen3ControllerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("gen3_controller", options),
      robot_(rlc_robot_models::MakeGen3Robot(
          rlc_robot_models::Gen3Variant::DOF7_VISION)) {
  // ---------- Parameters ----------
  controller_rate_hz_ =
      this->declare_parameter<double>("controller_rate_hz", 100.0);
  traj_delay_sec_ = this->declare_parameter<double>("traj_delay_sec", 0.0);

  if (!std::isfinite(controller_rate_hz_) || controller_rate_hz_ <= 0.0) {
    controller_rate_hz_ = 100.0;
  }
  controller_rate_hz_ = std::min(controller_rate_hz_, 1000.0);

  if (!std::isfinite(traj_delay_sec_) || traj_delay_sec_ < 0.0) {
    traj_delay_sec_ = 0.0;
  }

  // ---------- Robot Model ----------
  robot_.set_joint_prefix(joint_prefix_);
  rbt_core_cpp::Vec pose_wb(7);
  pose_wb << 1.0, 0.0, 0.75, 1.0, 0.0, 0.0, 0.0;
  robot_.set_base_pose_wb(pose_wb);
  robot_.ctrl().set_joint_gains(1.0, 1.0, 1.0);
  robot_.set_ctrl_mode(rbt_core_cpp::CtrlMode::CT);

  // ---------- Prealloc buffers ----------
  const int n = robot_.n();

  joint_state_rt_.resize(n);
  q_.setZero(n);
  qd_.setZero(n);
  q_des_.setZero(n);
  qd_des_.setZero(n);
  qdd_des_.setZero(n);

  snap_buf_[0].resize(n);
  snap_buf_[1].resize(n);

  effort_cmd_msg_.name.resize(n);
  effort_cmd_msg_.effort.resize(n);

  ctrl_state_msg_.joint_names.resize(n);
  ctrl_state_msg_.reference.positions.resize(n);
  ctrl_state_msg_.reference.velocities.resize(n);
  ctrl_state_msg_.reference.accelerations.resize(n);
  ctrl_state_msg_.feedback.positions.resize(n);
  ctrl_state_msg_.feedback.velocities.resize(n);
  ctrl_state_msg_.feedback.accelerations.resize(n);
  ctrl_state_msg_.error.positions.resize(n);
  ctrl_state_msg_.error.velocities.resize(n);
  ctrl_state_msg_.output.effort.resize(n);

  // ---------- Callback Groups ----------
  cb_group_state_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_services_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_action_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // ---------- Publishers ----------
  effort_pub_ = this->create_publisher<rlc_interfaces::msg::JointEffortCmd>(
      rlc_common::Topics().effort_cmd, qos_latest());
  ctrl_state_pub_ =
      this->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
          rlc_common::Topics().controller_state, qos_latest());

  // ---------- Subscribers ----------
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_state_;
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      rlc_common::Topics().joint_state, qos_latest(),
      std::bind(&Gen3ControllerNode::joint_state_callback_, this,
                std::placeholders::_1),
      sub_opts);

  // ---------- Services ----------
  set_gains_srv_ =
      this->create_service<rlc_interfaces::srv::SetControllerGains>(
          rlc_common::Services().set_controller_gains,
          std::bind(&Gen3ControllerNode::set_ctrl_gains_cb_, this,
                    std::placeholders::_1, std::placeholders::_2),
          rclcpp::ServicesQoS(), cb_group_services_);
  set_mode_srv_ = this->create_service<rlc_interfaces::srv::SetControllerMode>(
      rlc_common::Services().set_controller_mode,
      std::bind(&Gen3ControllerNode::set_ctrl_mode_cb_, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), cb_group_services_);

  // ---------- Action server ----------
  traj_action_server_ = rclcpp_action::create_server<FollowTraj>(
      this, rlc_common::Actions().follow_traj,
      std::bind(&Gen3ControllerNode::handle_traj_goal_, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&Gen3ControllerNode::handle_traj_cancel_, this,
                std::placeholders::_1),
      std::bind(&Gen3ControllerNode::handle_traj_accepted_, this,
                std::placeholders::_1),
      rcl_action_server_get_default_options(), cb_group_action_);

  // ---------- Control thread ----------
  running_.store(true, std::memory_order_release);
  control_thread_ = std::thread(&Gen3ControllerNode::control_loop_, this);

  RCLCPP_INFO(this->get_logger(),
              "Gen3 controller (C++) ready: n=%d, controller_rate=%.1f Hz, "
              "traj_delay=%.4f s",
              n, controller_rate_hz_, traj_delay_sec_);
}

Gen3ControllerNode::~Gen3ControllerNode() {
  running_.store(false, std::memory_order_release);

  if (control_thread_.joinable()) {
    control_thread_.join();
  }
}

std::string Gen3ControllerNode::normalize_joint_name_(
    const std::string& name, const std::string& prefix) {
  if (!prefix.empty() && name.rfind(prefix, 0) == 0) {
    return name.substr(prefix.size());
  }
  return name;
}

void Gen3ControllerNode::joint_state_callback_(
    const sensor_msgs::msg::JointState::SharedPtr msg) {

  if (joint_names_.empty()) {
    std::lock_guard<std::mutex> lk(joint_names_mutex_);
    joint_names_ = msg->name;
  }

  // Seqlock write: odd => writing, even => stable.
  const auto seq0 = joint_state_seq_.load(std::memory_order_relaxed);
  joint_state_seq_.store(seq0 + 1, std::memory_order_release);

  double stamp_sec = 0.0;
  const bool ok = ros_utils_cpp::msg_conv::ReadJointStateNoAlloc(
      *msg, joint_state_rt_.q, joint_state_rt_.qd, &stamp_sec);
  if (ok) {
    joint_state_rt_.t = stamp_sec;
    joint_state_rt_.valid = true;
  }

  joint_state_seq_.store(seq0 + 2, std::memory_order_release);
}

rclcpp_action::GoalResponse Gen3ControllerNode::handle_traj_goal_(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const FollowTraj::Goal> /*goal*/) {
  if (!running_.load(std::memory_order_acquire)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Gen3ControllerNode::handle_traj_cancel_(
    const std::shared_ptr<GoalHandleFollowTraj> /*goal_handle*/) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Gen3ControllerNode::handle_traj_accepted_(
    const std::shared_ptr<GoalHandleFollowTraj> goal_handle) {
  const auto seq = next_traj_seq_.fetch_add(1, std::memory_order_acq_rel);
  auto preempted = std::make_shared<std::atomic_bool>(false);

  {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    if (active_goal_.handle) {
      if (active_goal_.preempted) {
        active_goal_.preempted->store(true, std::memory_order_release);
      }
    }
    active_goal_.handle = goal_handle;
    active_goal_.preempted = preempted;
    active_goal_.seq = seq;
  }

  RCLCPP_INFO(this->get_logger(), "Accepted new trajectory goal (seq=%lu)",
              static_cast<unsigned long>(seq));

  auto self = std::dynamic_pointer_cast<Gen3ControllerNode>(shared_from_this());
  std::thread([self, goal_handle, preempted, seq]() {
    self->execute_follow_traj_(goal_handle, preempted, seq);
  }).detach();
}

void Gen3ControllerNode::execute_follow_traj_(
    std::shared_ptr<GoalHandleFollowTraj> goal_handle,
    std::shared_ptr<std::atomic_bool> preempted, std::uint64_t seq) {
  auto result = std::make_shared<FollowTraj::Result>();

  auto cleanup = [this, &goal_handle, seq]() {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    if (active_goal_.handle == goal_handle && active_goal_.seq == seq) {
      active_goal_ = ActiveGoalCtx{};
    }
  };

  const auto goal = goal_handle->get_goal();
  if (!goal) {
    result->error_code = FollowTraj::Result::INVALID_GOAL;
    result->error_string = "Null goal";
    goal_handle->abort(result);
    cleanup();
    return;
  }

  // Wait until robot I/O is configured (joint names known), but stay responsive
  // to cancel/preempt.
  while (running_.load(std::memory_order_acquire) && rclcpp::ok()) {
    if (preempted && preempted->load(std::memory_order_acquire)) {
      result->error_code = FollowTraj::Result::SUCCESSFUL;
      result->error_string = "Preempted by newer goal";
      goal_handle->abort(result);
      cleanup();
      return;
    }

    if (goal_handle->is_canceling()) {
      pending_traj_clear_seq_.store(seq, std::memory_order_release);
      result->error_code = FollowTraj::Result::SUCCESSFUL;
      result->error_string = "Trajectory cancelled";
      goal_handle->canceled(result);
      cleanup();
      return;
    }

    if (io_configured_.load(std::memory_order_acquire)) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (!running_.load(std::memory_order_acquire) || !rclcpp::ok()) {
    cleanup();
    return;
  }

  TrajCmd cmd;
  std::string err;
  if (!build_traj_cmd_(goal->trajectory, seq, &cmd, &err)) {
    result->error_code = FollowTraj::Result::INVALID_GOAL;
    result->error_string = err.empty() ? "Invalid trajectory" : err;
    goal_handle->abort(result);
    cleanup();
    return;
  }

  if (preempted && preempted->load(std::memory_order_acquire)) {
    result->error_code = FollowTraj::Result::SUCCESSFUL;
    result->error_string = "Preempted by newer goal";
    goal_handle->abort(result);
    cleanup();
    return;
  }

  if (goal_handle->is_canceling()) {
    pending_traj_clear_seq_.store(seq, std::memory_order_release);
    result->error_code = FollowTraj::Result::SUCCESSFUL;
    result->error_string = "Trajectory cancelled";
    goal_handle->canceled(result);
    cleanup();
    return;
  }

  std::atomic_store_explicit(&pending_traj_,
                             std::make_shared<const TrajCmd>(std::move(cmd)),
                             std::memory_order_release);

  // Wait for control thread to acknowledge applying this seq so we use the
  // correct finish time (based on the actual apply time).
  while (running_.load(std::memory_order_acquire) && rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      pending_traj_clear_seq_.store(seq, std::memory_order_release);
      result->error_code = FollowTraj::Result::SUCCESSFUL;
      result->error_string = "Trajectory cancelled";
      goal_handle->canceled(result);
      cleanup();
      return;
    }
    if (preempted && preempted->load(std::memory_order_acquire)) {
      result->error_code = FollowTraj::Result::SUCCESSFUL;
      result->error_string = "Preempted by newer goal";
      goal_handle->abort(result);
      cleanup();
      return;
    }

    if (active_traj_seq_.load(std::memory_order_acquire) == seq) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  const double tf = active_traj_tf_.load(std::memory_order_acquire);

  auto feedback = std::make_shared<FollowTraj::Feedback>();
  feedback->joint_names = joint_names_;
  feedback->actual.positions.resize(robot_.n());
  feedback->actual.velocities.resize(robot_.n());
  feedback->desired.positions.resize(robot_.n());
  feedback->desired.velocities.resize(robot_.n());
  feedback->error.positions.resize(robot_.n());

  CtrlSnapshot snap;
  snap.resize(robot_.n());

  const auto feedback_period = std::chrono::milliseconds(20);  // 50 Hz
  while (running_.load(std::memory_order_acquire) && rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      pending_traj_clear_seq_.store(seq, std::memory_order_release);
      result->error_code = FollowTraj::Result::SUCCESSFUL;
      result->error_string = "Trajectory cancelled";
      goal_handle->canceled(result);
      cleanup();
      return;
    }

    if (preempted && preempted->load(std::memory_order_acquire)) {
      result->error_code = FollowTraj::Result::SUCCESSFUL;
      result->error_string = "Preempted by newer goal";
      goal_handle->abort(result);
      cleanup();
      return;
    }

    snapshot_read_(&snap);

    feedback->header.stamp = ros_utils_cpp::time::ToRosTime(snap.t);
    for (int i = 0; i < robot_.n(); ++i) {
      feedback->actual.positions[i] = snap.q[i];
      feedback->actual.velocities[i] = snap.qd[i];
      feedback->desired.positions[i] = snap.q_des[i];
      feedback->desired.velocities[i] = snap.qd_des[i];
      feedback->error.positions[i] = snap.q_des[i] - snap.q[i];
    }
    goal_handle->publish_feedback(feedback);

    if (snap.t >= tf) break;
    std::this_thread::sleep_for(feedback_period);
  }

  result->error_code = FollowTraj::Result::SUCCESSFUL;
  result->error_string = "";
  goal_handle->succeed(result);
  cleanup();
}

void Gen3ControllerNode::set_ctrl_gains_cb_(
    const std::shared_ptr<rlc_interfaces::srv::SetControllerGains::Request>
        request,
    std::shared_ptr<rlc_interfaces::srv::SetControllerGains::Response>
        response) {
  const auto mode_raw = to_lower_copy(request->mode);

  GainsCmd tmp;
  tmp.kp = request->kp;
  tmp.kv = request->kv;
  tmp.ki = request->ki;

  if (mode_raw == "joint") {
    tmp.mode = GainMode::JOINT;
  } else if (mode_raw == "impedance") {
    tmp.mode = GainMode::CARTESIAN;
  } else {
    response->success = false;
    response->message = "Unknown gain mode '" + request->mode +
                        "'. Expected 'joint' or 'impedance'.";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  std::atomic_store_explicit(&pending_gains_,
                             std::make_shared<const GainsCmd>(tmp),
                             std::memory_order_release);

  response->success = true;
  response->message =
      "Requested to set " + mode_raw + " gains: Kp=" + std::to_string(tmp.kp) +
      ", Kv=" + std::to_string(tmp.kv) + ", Ki=" + std::to_string(tmp.ki);
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void Gen3ControllerNode::set_ctrl_mode_cb_(
    const std::shared_ptr<rlc_interfaces::srv::SetControllerMode::Request>
        request,
    std::shared_ptr<rlc_interfaces::srv::SetControllerMode::Response>
        response) {
  const auto mode_raw = to_lower_copy(request->mode);
  ModeCmd tmp;

  if (mode_raw == "pid") {
    tmp.mode = rbt_core_cpp::CtrlMode::PID;
  } else if (mode_raw == "ct") {
    tmp.mode = rbt_core_cpp::CtrlMode::CT;
  } else if (mode_raw == "im") {
    tmp.mode = rbt_core_cpp::CtrlMode::IM;
  } else if (mode_raw == "gc") {
    tmp.mode = rbt_core_cpp::CtrlMode::GC;
  } else {
    response->success = false;
    response->message = "Unknown control mode '" + request->mode +
                        "'. Expected 'pid', 'ct', 'im', or 'gc'.";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  std::atomic_store_explicit(&pending_mode_,
                             std::make_shared<const ModeCmd>(tmp),
                             std::memory_order_release);

  response->success = true;
  response->message = "Requested to set control mode to '" + mode_raw + "'";
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

bool Gen3ControllerNode::try_configure_robot_io_() {
  if (io_configured_.load(std::memory_order_acquire)) return true;

  std::vector<std::string> names;
  {
    std::lock_guard<std::mutex> lk(joint_names_mutex_);
    if (joint_names_.empty()) return false;
    names = joint_names_;
  }

  try {
    robot_.configure_io(names, joint_prefix_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Robot I/O configure failed: %s",
                 e.what());
    return false;
  }

  // Build model->src index map for fast desired conversion.
  const auto& model_names = robot_.dyn().canonical_joint_names();
  const int n = robot_.n();

  std::unordered_map<std::string, int> src_index;
  src_index.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    src_index.emplace(normalize_joint_name_(names[i], joint_prefix_), i);
  }

  std::vector<int> src_to_model(n, 0);
  for (int model_i = 0; model_i < n; ++model_i) {
    const auto it = src_index.find(model_names[model_i]);
    if (it == src_index.end()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Robot I/O configure: missing source joint '%s'",
                   model_names[model_i].c_str());
      return false;
    }
    src_to_model[model_i] = it->second;
  }

  model_to_src_idx_.assign(n, 0);
  for (int model_i = 0; model_i < n; ++model_i) {
    model_to_src_idx_[src_to_model[model_i]] = model_i;
  }

  // Finalize message joint names once.
  effort_cmd_msg_.name = names;
  ctrl_state_msg_.joint_names = names;

  io_configured_.store(true, std::memory_order_release);
  RCLCPP_INFO(this->get_logger(), "Robot I/O configured.");
  return true;
}

bool Gen3ControllerNode::build_traj_cmd_(
    const trajectory_msgs::msg::JointTrajectory& msg, std::uint64_t seq,
    TrajCmd* out_cmd, std::string* out_err) const {
  if (!out_cmd) return false;
  if (msg.points.empty()) {
    if (out_err) *out_err = "Empty trajectory";
    return false;
  }

  const int n = robot_.n();
  if (static_cast<int>(msg.joint_names.size()) != n) {
    if (out_err) {
      *out_err = "Trajectory joint_names size mismatch (got " +
                 std::to_string(msg.joint_names.size()) + ", expected " +
                 std::to_string(n) + ")";
    }
    return false;
  }

  if (static_cast<int>(joint_names_.size()) != n) {
    if (out_err) *out_err = "Robot joint names not initialized";
    return false;
  }

  std::unordered_map<std::string, int> goal_index;
  goal_index.reserve(static_cast<std::size_t>(n));
  for (int j = 0; j < n; ++j) {
    goal_index.emplace(normalize_joint_name_(msg.joint_names[j], joint_prefix_),
                       j);
  }

  std::vector<int> goal_idx_of_src(n, 0);
  for (int src_i = 0; src_i < n; ++src_i) {
    const auto key = normalize_joint_name_(joint_names_[src_i], joint_prefix_);
    const auto it = goal_index.find(key);
    if (it == goal_index.end()) {
      if (out_err) {
        *out_err = "Trajectory missing joint '" + joint_names_[src_i] + "'";
      }
      return false;
    }
    goal_idx_of_src[src_i] = it->second;
  }

  const int N = static_cast<int>(msg.points.size());
  out_cmd->traj.t.resize(N);
  out_cmd->traj.q.resize(N, n);
  out_cmd->traj.qd.resize(N, n);
  out_cmd->traj.qdd.resize(N, n);

  double prev_t = -std::numeric_limits<double>::infinity();
  for (int k = 0; k < N; ++k) {
    const auto& pt = msg.points[static_cast<std::size_t>(k)];
    const double t = ros_utils_cpp::time::ToSec(pt.time_from_start);
    if (!(t > prev_t)) {
      if (out_err) *out_err = "time_from_start must be strictly increasing";
      return false;
    }
    prev_t = t;
    out_cmd->traj.t[k] = t;

    if (static_cast<int>(pt.positions.size()) != n) {
      if (out_err) *out_err = "Trajectory point positions size mismatch";
      return false;
    }

    const bool has_vel = !pt.velocities.empty();
    const bool has_acc = !pt.accelerations.empty();

    if (has_vel && static_cast<int>(pt.velocities.size()) != n) {
      if (out_err) *out_err = "Trajectory point velocities size mismatch";
      return false;
    }
    if (has_acc && static_cast<int>(pt.accelerations.size()) != n) {
      if (out_err) *out_err = "Trajectory point accelerations size mismatch";
      return false;
    }

    for (int src_i = 0; src_i < n; ++src_i) {
      const int j = goal_idx_of_src[src_i];
      out_cmd->traj.q(k, src_i) = pt.positions[static_cast<std::size_t>(j)];
      out_cmd->traj.qd(k, src_i) =
          has_vel ? pt.velocities[static_cast<std::size_t>(j)] : 0.0;
      out_cmd->traj.qdd(k, src_i) =
          has_acc ? pt.accelerations[static_cast<std::size_t>(j)] : 0.0;
    }
  }

  out_cmd->delay_sec = traj_delay_sec_;
  out_cmd->duration_sec = out_cmd->traj.t[N - 1];
  out_cmd->seq = seq;
  return true;
}

void Gen3ControllerNode::snapshot_write_() {
  const int write_idx = 1 - snap_read_idx_.load(std::memory_order_relaxed);
  auto& s = snap_buf_[write_idx];

  s.t = robot_.t();
  s.q = q_;
  s.qd = qd_;
  s.q_des = q_des_src_;
  s.qd_des = qd_des_src_;

  snap_read_idx_.store(write_idx, std::memory_order_release);
}

void Gen3ControllerNode::snapshot_read_(CtrlSnapshot* out) const {
  if (!out) return;
  const int idx = snap_read_idx_.load(std::memory_order_acquire);
  *out = snap_buf_[idx];
}

void Gen3ControllerNode::control_loop_() {
  using clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(1.0 / controller_rate_hz_));
  auto next = clock::now();

  const int n = robot_.n();

  while (running_.load(std::memory_order_acquire)) {
    next += period;

    // Read latest joint state (lock-free seqlock).
    bool got = false;
    for (int attempt = 0; attempt < 3 && !got; ++attempt) {
      const auto s1 = joint_state_seq_.load(std::memory_order_acquire);
      if (s1 & 1U) continue;  // writer active
      if (!joint_state_rt_.valid) break;

      q_ = joint_state_rt_.q;
      qd_ = joint_state_rt_.qd;
      const double t = joint_state_rt_.t;

      const auto s2 = joint_state_seq_.load(std::memory_order_acquire);
      if (s1 == s2) {
        // Configure I/O once we have names.
        if (!io_configured_.load(std::memory_order_acquire)) {
          (void)try_configure_robot_io_();
        }

        if (io_configured_.load(std::memory_order_acquire)) {
          robot_.set_joint_state(&q_, &qd_, nullptr, &t);
        }
        got = true;
      }
    }

    if (!got || !io_configured_.load(std::memory_order_acquire)) {
      std::this_thread::sleep_until(next);
      continue;
    }

    // Consume pending clear request (seq-scoped).
    const auto clear_seq =
        pending_traj_clear_seq_.exchange(0, std::memory_order_acq_rel);

    // Drop a pending trajectory cmd if it matches a cancel request.
    if (clear_seq != 0) {
      auto pending =
          std::atomic_load_explicit(&pending_traj_, std::memory_order_acquire);
      if (pending && pending->seq == clear_seq) {
        std::atomic_store_explicit(&pending_traj_,
                                   std::shared_ptr<const TrajCmd>(),
                                   std::memory_order_release);
      }
      if (active_traj_seq_.load(std::memory_order_acquire) == clear_seq) {
        robot_.clear_traj();
        active_traj_seq_.store(0, std::memory_order_release);
        active_traj_tf_.store(0.0, std::memory_order_release);
      }
    }

    // Consume mode cmd
    if (auto cmd = std::atomic_exchange_explicit(
            &pending_mode_, std::shared_ptr<const ModeCmd>(),
            std::memory_order_acq_rel)) {
      robot_.set_ctrl_mode(cmd->mode);
      RCLCPP_INFO(this->get_logger(), "Set control mode");
    }

    // Consume gains cmd
    if (auto cmd = std::atomic_exchange_explicit(
            &pending_gains_, std::shared_ptr<const GainsCmd>(),
            std::memory_order_acq_rel)) {
      if (cmd->mode == GainMode::JOINT) {
        robot_.ctrl().set_joint_gains(cmd->kp, cmd->kv, cmd->ki);
        RCLCPP_INFO(this->get_logger(),
                    "Set joint gains: Kp=%.3f Kv=%.3f Ki=%.3f", cmd->kp,
                    cmd->kv, cmd->ki);
      } else {
        robot_.ctrl().set_task_gains(cmd->kp, cmd->kv, cmd->ki);
        RCLCPP_INFO(this->get_logger(),
                    "Set Cartesian gains: Kx=%.3f Dx=%.3f Kix=%.3f", cmd->kp,
                    cmd->kv, cmd->ki);
      }
    }

    // Consume trajectory cmd (latest-wins).
    if (auto cmd = std::atomic_exchange_explicit(
            &pending_traj_, std::shared_ptr<const TrajCmd>(),
            std::memory_order_acq_rel)) {
      // If this seq was cancelled before apply, drop it.
      if (cmd->seq == clear_seq) {
        // nothing
      } else {
        try {
          robot_.set_joint_traj(cmd->traj, cmd->delay_sec);
          const double tf = robot_.t() + cmd->delay_sec + cmd->duration_sec;
          active_traj_seq_.store(cmd->seq, std::memory_order_release);
          active_traj_tf_.store(tf, std::memory_order_release);
          RCLCPP_INFO(this->get_logger(),
                      "Set joint trajectory (seq=%lu, duration=%.3f s, "
                      "delay=%.4f s)",
                      static_cast<unsigned long>(cmd->seq), cmd->duration_sec,
                      cmd->delay_sec);
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "set_joint_traj failed: %s",
                       e.what());
        }
      }
    }

    // Compute control
    const auto& tau_src = robot_.compute_ctrl_effort();

    // Desired: model -> src for feedback/state topics.
    const auto& q_des_model = robot_.q_des_model();
    const auto& qd_des_model = robot_.qd_des_model();
    const auto& qdd_des_model = robot_.qdd_des_model();

    for (int src_i = 0; src_i < n; ++src_i) {
      const int model_i = model_to_src_idx_[src_i];
      q_des_src_[src_i] = q_des_model[model_i];
      qd_des_src_[src_i] = qd_des_model[model_i];
      qdd_des_src_[src_i] = qdd_des_model[model_i];
    }

    // Publish effort cmd (reuse message, no resizing).
    (void)ros_utils_cpp::msg_conv::WriteJointEffortCmdNoAlloc(
        robot_.t(), tau_src, &effort_cmd_msg_);
    effort_pub_->publish(effort_cmd_msg_);

    // Publish controller state (reuse message, no resizing).
    (void)ros_utils_cpp::msg_conv::WriteJointCtrlStateNoAlloc(
        robot_.t(), tau_src, q_, qd_, q_des_src_, qd_des_src_, qdd_des_src_,
        &ctrl_state_msg_);
    ctrl_state_pub_->publish(ctrl_state_msg_);

    snapshot_write_();
    std::this_thread::sleep_until(next);
  }
}

}  // namespace rlc_controller_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rlc_controller_cpp::Gen3ControllerNode>();

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  exec.spin();

  exec.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
