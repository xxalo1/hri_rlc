#include "rlc_controller_cpp/realtime_controller.hpp"

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <exception>
#include <rbt_core_cpp/robot.hpp>
#include <rlc_utils/types.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rlc_utils/msg_conv.hpp"
#include "rlc_utils/time_util.hpp"

namespace rlc_controller_cpp {

namespace rumsg = rlc_utils::msg_conv;
namespace rutypes = rlc_utils::types;
namespace rytime = rlc_utils::time_util;
namespace ci = controller_interface;
using Self = RealtimeController;
using CallbackReturn = ci::CallbackReturn;
using FollowJTAction = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJTAction>;

RealtimeController::RealtimeController() = default;

ci::CallbackReturn Self::on_init() {
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("tcp_frame", "tool_frame");

  return ci::CallbackReturn::SUCCESS;
}

ci::InterfaceConfiguration Self::command_interface_configuration() const {
  ci::InterfaceConfiguration cfg;
  cfg.type = ci::interface_configuration_type::INDIVIDUAL;
  cfg.names = claimed_command_interfaces_;
  return cfg;
}

ci::InterfaceConfiguration Self::state_interface_configuration() const {
  ci::InterfaceConfiguration cfg;
  cfg.type = ci::interface_configuration_type::INDIVIDUAL;
  cfg.names = claimed_state_interfaces_;
  return cfg;
}

ci::CallbackReturn Self::on_configure(const rclcpp_lifecycle::State&) {
  joints_ = get_node()->get_parameter("joints").as_string_array();
  const auto tcp_frame = get_node()->get_parameter("tcp_frame").as_string();

  if (joints_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Parameter 'joints' must be set and non-empty");
    return ci::CallbackReturn::FAILURE;
  }

  auto urdf = get_robot_description();

  if (urdf.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "ParaFailed to get URDF from /robot_description (timeout).");
    return ci::CallbackReturn::FAILURE;
  }

  set_ndof(joints_.size());
  const std::size_t n = ndof();

  // Pre-size RT snapshot storage so update() stays allocation-free.
  snapshot_rt_.resize(n);
  {
    Self::JointSnapshot init(n);
    snapshot_box_.set(init);
  }

  claimed_state_interfaces_.clear();
  claimed_command_interfaces_.clear();
  claimed_state_interfaces_.reserve(n * 2);
  claimed_command_interfaces_.reserve(n);

  for (const auto& j : joints_) {
    claimed_state_interfaces_.emplace_back(j + "/" +
                                           hardware_interface::HW_IF_POSITION);
    claimed_state_interfaces_.emplace_back(j + "/" +
                                           hardware_interface::HW_IF_VELOCITY);
    claimed_command_interfaces_.emplace_back(j + "/" +
                                             hardware_interface::HW_IF_EFFORT);
  }

  rbt_core_cpp::RobotSpec spec{
      .name = "rlc_robot",
      .urdf = std::move(urdf),
      .tcp_frame = tcp_frame,
      .urdf_source = rbt_core_cpp::UrdfSource::Xml,
  };

  try {
    robot_.emplace(spec);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to construct robot model: %s", e.what());
    return ci::CallbackReturn::FAILURE;
  }

  const auto n_model = static_cast<std::size_t>(robot_->n());
  if (n_model != n) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Robot DOF mismatch: robot.n()=%zu, joints.size()=%zu",
                 n_model, n);
    return ci::CallbackReturn::FAILURE;
  }

  const Eigen::Index ni = static_cast<Eigen::Index>(n);

  joint_state_.resize(ni);
  joint_cmd_.resize(ni);

  joint_state_.name = joints_;
  joint_cmd_.name = joints_;

  try {
    robot_->configure_io(joints_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_.configure_io failed: %s",
                 e.what());
    return ci::CallbackReturn::FAILURE;
  }

  return ci::CallbackReturn::SUCCESS;
}

ci::CallbackReturn Self::init_interface_handles() {
  const std::size_t n = ndof();

  state_handles_.assign(n, Self::JointStateInterfaces{});
  command_handles_.assign(n, Self::JointCommandInterfaces{});

  auto find_state = [this](const std::string& name)
      -> hardware_interface::LoanedStateInterface* {
    for (auto& si : state_interfaces_) {
      if (si.get_name() == name) return &si;
    }
    return nullptr;
  };

  auto find_command = [this](const std::string& name)
      -> hardware_interface::LoanedCommandInterface* {
    for (auto& ci : command_interfaces_) {
      if (ci.get_name() == name) return &ci;
    }
    return nullptr;
  };

  for (std::size_t i = 0; i < n; ++i) {
    const auto& position_name = claimed_state_interfaces_[2 * i];
    const auto& velocity_name = claimed_state_interfaces_[2 * i + 1];
    const auto& command_name = claimed_command_interfaces_[i];

    state_handles_[i].position = find_state(position_name);
    if (!state_handles_[i].position) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface: %s",
                   position_name.c_str());
      return ci::CallbackReturn::FAILURE;
    }

    state_handles_[i].velocity = find_state(velocity_name);
    if (!state_handles_[i].velocity) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface: %s",
                   velocity_name.c_str());
      return ci::CallbackReturn::FAILURE;
    }

    command_handles_[i].effort = find_command(command_name);
    if (!command_handles_[i].effort) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing command interface: %s",
                   command_name.c_str());
      return ci::CallbackReturn::FAILURE;
    }
  }

  return ci::CallbackReturn::SUCCESS;
}

ci::CallbackReturn Self::calibrate() {
  const rclcpp::Time time = get_node()->now();
  const auto rc = read_state(time);
  if (rc != ci::return_type::OK) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to read state during calibration");
    return ci::CallbackReturn::FAILURE;
  }
  robot_->set_joint_des(&joint_state_.position);
  robot_->set_joint_state(&joint_state_.position, &joint_state_.velocity,
                          nullptr, &joint_state_.stamp_sec);
  joint_cmd_.effort = robot_->compute_ctrl_effort();

  joint_cmd_.stamp_sec = joint_state_.stamp_sec;

  const auto rc_write = write_cmd();
  if (rc_write != ci::return_type::OK) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to write command during calibration");
    return ci::CallbackReturn::FAILURE;
  }

  return ci::CallbackReturn::SUCCESS;
}

ci::CallbackReturn Self::on_activate(const rclcpp_lifecycle::State&) {
  is_active_.store(true, std::memory_order_release);

  auto rc = init_interface_handles();
  if (rc != ci::CallbackReturn::SUCCESS) {
    is_active_.store(false, std::memory_order_release);
    return rc;
  }

  rc = calibrate();
  if (rc != ci::CallbackReturn::SUCCESS) {
    is_active_.store(false, std::memory_order_release);
    return rc;
  }

  return ci::CallbackReturn::SUCCESS;
}

ci::CallbackReturn Self::on_deactivate(const rclcpp_lifecycle::State&) {
  is_active_.store(false, std::memory_order_release);

  state_handles_.clear();
  command_handles_.clear();
  return ci::CallbackReturn::SUCCESS;
}

ci::return_type Self::update(const rclcpp::Time& time,
                             const rclcpp::Duration&) {
  auto rt = read_state(time);
  if (rt != ci::return_type::OK) {
    return rt;
  }

  consume_pending_traj_rt();

  robot_->set_joint_state(&joint_state_.position, &joint_state_.velocity,
                          nullptr, &joint_state_.stamp_sec);
  joint_cmd_.effort.noalias() = robot_->compute_ctrl_effort();
  joint_cmd_.stamp_sec = joint_state_.stamp_sec;

  // Snapshot after robot state/desired are updated for this cycle.
  update_snapshot_rt();

  rt = write_cmd();
  if (rt != ci::return_type::OK) {
    return rt;
  }

  return ci::return_type::OK;
}

// --------------------------------------------------------
// RT helpers
// --------------------------------------------------------

ci::return_type Self::read_state(const rclcpp::Time& time) {
  const std::size_t n = ndof();
  auto* q = joint_state_.position.data();
  auto* qd = joint_state_.velocity.data();
  for (std::size_t i = 0; i < n; ++i) {
    const auto pos = state_handles_[i].position->get_optional<double>(1);
    const auto vel = state_handles_[i].velocity->get_optional<double>(1);
    if (!pos || !vel) return ci::return_type::ERROR;
    q[i] = *pos;
    qd[i] = *vel;
  }
  joint_state_.stamp_sec = time.seconds();
  return ci::return_type::OK;
}

ci::return_type Self::write_cmd() {
  const std::size_t n = ndof();
  auto* effort = joint_cmd_.effort.data();
  for (std::size_t i = 0; i < n; ++i) {
    const bool success = command_handles_[i].effort->set_value(effort[i], 1);
    if (!success) return ci::return_type::ERROR;
  }
  return ci::return_type::OK;
}

void Self::consume_pending_traj_rt() {
  const auto* slot = pending_traj_.readFromRT();
  const auto& traj_ptr = *slot;
  if (!traj_ptr) return;

  if (traj_ptr->seq == last_applied_traj_seq_) return;

  last_applied_traj_seq_ = traj_ptr->seq;

  robot_->set_joint_traj(traj_ptr->traj);
}

void Self::update_snapshot_rt() {
  const std::size_t n = ndof();
  const double stamp_sec = joint_state_.stamp_sec;
  const uint64_t active_traj_seq = last_applied_traj_seq_;
  const double* q_src = joint_state_.position.data();
  const double* qd_src = joint_state_.velocity.data();
  const double* q_des_src = robot_->q_des_model().data();
  const double* qd_des_src = robot_->qd_des_model().data();

  snapshot_box_.try_set([&](Self::JointSnapshot& s) {
    s.t = stamp_sec;
    s.active_traj_seq = active_traj_seq;
    std::memcpy(s.q.data(), q_src, n * sizeof(double));
    std::memcpy(s.qd.data(), qd_src, n * sizeof(double));
    std::memcpy(s.q_des.data(), q_des_src, n * sizeof(double));
    std::memcpy(s.qd_des.data(), qd_des_src, n * sizeof(double));
  });
}

// ------------------------------------------------
// Joint Trajectory Action Server callback functions
// ------------------------------------------------

rclcpp_action::GoalResponse Self::handle_traj_goal(
    const rclcpp_action::GoalUUID&,
  std::shared_ptr<const FollowJTAction::Goal> goal) {
  if (!is_active_.load()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!goal) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto& jt = goal->trajectory;
  if (jt.joint_names.empty() || jt.points.empty()) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Validate that the goal includes all controller joints.
  std::unordered_map<std::string, std::size_t> idx;
  idx.reserve(jt.joint_names.size());
  for (std::size_t i = 0; i < jt.joint_names.size(); ++i) {
    idx.emplace(jt.joint_names[i], i);
  }
  for (const auto& j : joints_) {
    if (idx.find(j) == idx.end()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Rejecting trajectory goal: missing joint '%s'", j.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Self::handle_traj_cancel(
    const std::shared_ptr<GoalHandle>) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Self::handle_traj_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
  if (!goal_handle) return;

  // Preempt any existing goal.
  auto preempt_flag = std::make_shared<std::atomic_bool>(false);

  {
    std::lock_guard<std::mutex> lk(action_mutex_);
    if (active_preempt_flag_) {
      active_preempt_flag_->store(true);
    }
    active_goal_ = goal_handle;
    active_preempt_flag_ = preempt_flag;
  }

  // Run goal execution on a separate (non-RT) thread.
  std::lock_guard<std::mutex> lk(worker_mutex_);
  worker_threads_.emplace_back([this, goal_handle, preempt_flag]() mutable {
    this->execute_traj_goal(std::move(goal_handle), std::move(preempt_flag));
  });
}

// ------------------------------------------------
// Joint Trajectory Action Server helper functions
// ------------------------------------------------

ci::CallbackReturn Self::init_traj_action_server() {
  const auto node = get_node();
  if (!node) {
    return ci::CallbackReturn::FAILURE;
  }

  using namespace std::placeholders;

  traj_action_server_ = rclcpp_action::create_server<FollowJTAction>(
      node, "~/follow_joint_trajectory",
      std::bind(&Self::handle_traj_goal, this, _1, _2),
      std::bind(&Self::handle_traj_cancel, this, _1),
      std::bind(&Self::handle_traj_accepted, this, _1));

  RCLCPP_INFO(get_node()->get_logger(),
              "FollowJointTrajectory action server ready on '%s'",
              "~/follow_joint_trajectory");

  return ci::CallbackReturn::SUCCESS;
}

bool Self::build_traj_command(const trajectory_msgs::msg::JointTrajectory& jt,
                              rutypes::JointTrajectoryMsgData& msg_data,
                              std::string* error_string) {
  const std::size_t n = ndof();
  const std::size_t n_in = jt.joint_names.size();
  const std::size_t N = jt.points.size();

  if (n_in == 0 || N == 0) {
    if (error_string) *error_string = "Empty trajectory";
    return false;
  }

  if (n_in != n) {
    if (error_string) {
      *error_string = "Joint count mismatch: expected " + std::to_string(n) +
                      ", got " + std::to_string(n_in);
    }
    return false;
  }

  for (std::size_t i = 0; i < n; ++i) {
    if (joints_[i] != jt.joint_names[i]) {
      if (error_string) {
        *error_string = "Joint name mismatch at index " + std::to_string(i) +
                        ": expected '" + joints_[i] + "', got '" +
                        jt.joint_names[i] + "'";
      }
      return false;
    }
  }

  msg_data.resize(N, n);
  msg_data.joint_names = joints_;
  const bool ok = rumsg::from_joint_trajectory_msg(jt, msg_data);

  if (!ok) {
    if (error_string) {
      *error_string =
          "Invalid trajectory: require strictly increasing time_from_start "
          "and "
          "positions/velocities/accelerations for every point";
    }
    return false;
  }

  return true;
}

void Self::execute_traj_goal(std::shared_ptr<GoalHandle> goal_handle,
                             std::shared_ptr<std::atomic_bool> preempt_flag) {
  auto result = std::make_shared<FollowJTAction::Result>();

  if (!goal_handle || !preempt_flag) {
    return;
  }

  const auto goal = goal_handle->get_goal();
  const auto& jt = goal->trajectory;

  std::string err;
  rutypes::JointTrajectoryMsgData msg_data;
  auto ok = build_traj_command(jt, msg_data, &err);
  if (!ok) {
    result->error_code = FollowJTAction::Result::INVALID_GOAL;
    result->error_string = err;
    goal_handle->abort(result);
    return;
  }

  auto traj = std::make_shared<rbt_core_cpp::JointTrajectory>();

  traj->t = std::move(msg_data.t);
  traj->q = std::move(msg_data.q);
  traj->qd = std::move(msg_data.qd);
  traj->qdd = std::move(msg_data.qdd);

  auto pending = std::make_shared<Self::PendingJointTrajectory>();
  pending->traj = traj;

  static std::atomic_uint64_t seq{1};
  pending->seq = seq.fetch_add(1);

  pending_traj_.writeFromNonRT(pending);

  FollowJTAction::Feedback feedback;
  feedback.joint_names = joints_;

  const double feedback_hz = 50.0;
  const double feedback_dt = 1.0 / feedback_hz;

  const std::size_t n = ndof();

  const double tf = traj->t(traj->length() - 1);

  Self::JointSnapshot snap(n);

  double t0;
  snapshot_box_.get([&](const Self::JointSnapshot& s) {
    t0 = s.t;
    snap.t = s.t;
    snap.active_traj_seq = s.active_traj_seq;
    for (std::size_t i = 0; i < n; ++i) {
      snap.q[i] = s.q[i];
      snap.qd[i] = s.qd[i];
      snap.q_des[i] = s.q_des[i];
      snap.qd_des[i] = s.qd_des[i];
    }
  });

  while (rclcpp::ok() && !shutting_down_.load()) {
    if (goal_handle->is_canceling()) {
      result->error_code = FollowJTAction::Result::SUCCESSFUL;
      result->error_string = "Trajectory cancelled";
      goal_handle->canceled(result);
      return;
    }

    if (preempt_flag->load()) {
      result->error_code = FollowJTAction::Result::SUCCESSFUL;
      result->error_string = "Preempted by a newer goal";
      goal_handle->abort(result);
      return;
    }

    (void)snapshot_box_.try_get([&](const Self::JointSnapshot& s) {
      snap.t = s.t;
      snap.active_traj_seq = s.active_traj_seq;
      for (std::size_t i = 0; i < n; ++i) {
        snap.q[i] = s.q[i];
        snap.qd[i] = s.qd[i];
        snap.q_des[i] = s.q_des[i];
        snap.qd_des[i] = s.qd_des[i];
      }
    });

    const double t_sec = snap.t;

    feedback.header.stamp = rytime::to_ros_time(t_sec);
    feedback.actual.positions = snap.q;
    feedback.actual.velocities = snap.qd;
    feedback.desired.positions = snap.q_des;
    feedback.desired.velocities = snap.qd_des;

    feedback.error.positions.resize(snap.q.size(), 0.0);
    feedback.error.velocities.resize(snap.q.size(), 0.0);
    for (std::size_t i = 0; i < snap.q.size(); ++i) {
      feedback.error.positions[i] = snap.q_des[i] - snap.q[i];
      feedback.error.velocities[i] = snap.qd_des[i] - snap.qd[i];
    }

    goal_handle->publish_feedback(
      std::make_shared<FollowJTAction::Feedback>(feedback));

    if (t_sec >= tf) {
      break;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(feedback_dt));
  }

  result->error_code = FollowJTAction::Result::SUCCESSFUL;
  result->error_string = "Trajectory succeeded";
  goal_handle->succeed(result);
  return;
}

}  // namespace rlc_controller_cpp

PLUGINLIB_EXPORT_CLASS(rlc_controller_cpp::RealtimeController,
                       controller_interface::ControllerInterface)
