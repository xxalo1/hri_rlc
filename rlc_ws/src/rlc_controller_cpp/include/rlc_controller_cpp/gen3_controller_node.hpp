#pragma once

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rlc_interfaces/msg/joint_effort_cmd.hpp>
#include <rlc_interfaces/srv/set_controller_gains.hpp>
#include <rlc_interfaces/srv/set_controller_mode.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rbt_core_cpp/robot.hpp"

namespace rlc_controller_cpp {

class Gen3ControllerNode final : public rclcpp::Node {
 public:
  explicit Gen3ControllerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~Gen3ControllerNode() override;

 private:
  using FollowTraj = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowTraj = rclcpp_action::ServerGoalHandle<FollowTraj>;

  enum class GainMode { JOINT, CARTESIAN };

  struct JointStateBuf {
    rbt_core_cpp::Vec q;
    rbt_core_cpp::Vec qd;
    double t{0.0};
    bool valid{false};

    void resize(int n) {
      q.setZero(n);
      qd.setZero(n);
    }
  };

  struct DesiredBuf {
    rbt_core_cpp::Vec q_des;
    rbt_core_cpp::Vec qd_des;
    rbt_core_cpp::Vec qdd_des;

    void resize(int n) {
      q_des.setZero(n);
      qd_des.setZero(n);
      qdd_des.setZero(n);
    }
  };

  struct CtrlSnapshot {
    double t{0.0};
    rbt_core_cpp::Vec q;
    rbt_core_cpp::Vec qd;
    rbt_core_cpp::Vec q_des;
    rbt_core_cpp::Vec qd_des;

    void resize(int n) {
      q.setZero(n);
      qd.setZero(n);
      q_des.setZero(n);
      qd_des.setZero(n);
    }
  };

  struct GainsCmd {
    double kp{0.0};
    double kv{0.0};
    double ki{0.0};
    GainMode mode{GainMode::JOINT};
  };

  struct ModeCmd {
    rbt_core_cpp::CtrlMode mode{rbt_core_cpp::CtrlMode::CT};
  };

  struct TrajCmd {
    rbt_core_cpp::JointTrajectory traj;
    double delay_sec{0.0};
    double duration_sec{0.0};
    std::uint64_t seq{0};
  };

  struct ActiveGoalCtx {
    std::shared_ptr<GoalHandleFollowTraj> handle;
    std::shared_ptr<std::atomic_bool> preempted;
    std::uint64_t seq{0};
  };

  // ---------- ROS callbacks ----------
  void joint_state_callback_(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp_action::GoalResponse handle_traj_goal_(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const FollowTraj::Goal> goal);
  rclcpp_action::CancelResponse handle_traj_cancel_(
      const std::shared_ptr<GoalHandleFollowTraj> goal_handle);
  void handle_traj_accepted_(const std::shared_ptr<GoalHandleFollowTraj> goal_handle);

  void execute_follow_traj_(std::shared_ptr<GoalHandleFollowTraj> goal_handle,
                            std::shared_ptr<std::atomic_bool> preempted,
                            std::uint64_t seq);

  void set_ctrl_gains_cb_(
      const std::shared_ptr<rlc_interfaces::srv::SetControllerGains::Request>
          request,
      std::shared_ptr<rlc_interfaces::srv::SetControllerGains::Response>
          response);
  void set_ctrl_mode_cb_(
      const std::shared_ptr<rlc_interfaces::srv::SetControllerMode::Request>
          request,
      std::shared_ptr<rlc_interfaces::srv::SetControllerMode::Response>
          response);

  // ---------- Control loop ----------
  void control_loop_();
  bool try_configure_robot_io_();

  // ---------- Helpers ----------
  static std::string normalize_joint_name_(const std::string& name,
                                          const std::string& prefix);

  bool build_traj_cmd_(const trajectory_msgs::msg::JointTrajectory& msg,
                       std::uint64_t seq, TrajCmd* out_cmd,
                       std::string* out_err) const;

  void snapshot_write_();
  void snapshot_read_(CtrlSnapshot* out) const;

  // ---------- Threads / lifecycle ----------
  std::atomic_bool running_{false};
  std::thread control_thread_;

  // ---------- Parameters ----------
  double controller_rate_hz_{100.0};
  double traj_delay_sec_{0.0};

  // ---------- Robot ----------
  rbt_core_cpp::Robot robot_;
  std::atomic_bool io_configured_{false};

  std::vector<std::string> joint_names_;
  std::string joint_prefix_{"gen3_"};
  std::vector<int> model_to_src_idx_;

  mutable std::mutex joint_names_mutex_;

  // ---------- Joint state (writer: ROS thread, reader: control thread) ----------
  JointStateBuf joint_state_rt_;
  mutable std::atomic<std::uint64_t> joint_state_seq_{0};

  // Preallocated working buffers (control thread only)
  rbt_core_cpp::Vec q_src_;
  rbt_core_cpp::Vec qd_src_;
  rbt_core_cpp::Vec q_des_src_;
  rbt_core_cpp::Vec qd_des_src_;
  rbt_core_cpp::Vec qdd_des_src_;

  // ---------- Snapshot (writer: control thread, reader: action threads) ----------
  mutable std::atomic<int> snap_read_idx_{0};
  CtrlSnapshot snap_buf_[2];

  // ---------- Pending commands (callbacks -> control thread) ----------
  std::shared_ptr<const GainsCmd> pending_gains_;
  std::shared_ptr<const ModeCmd> pending_mode_;
  std::shared_ptr<const TrajCmd> pending_traj_;

  std::atomic<std::uint64_t> next_traj_seq_{1};
  std::atomic<std::uint64_t> active_traj_seq_{0};
  std::atomic<double> active_traj_tf_{0.0};
  std::atomic<std::uint64_t> pending_traj_clear_seq_{0};

  // ---------- Action state ----------
  mutable std::mutex goal_mutex_;
  ActiveGoalCtx active_goal_;

  // ---------- ROS interfaces ----------
  rclcpp::CallbackGroup::SharedPtr cb_group_state_;
  rclcpp::CallbackGroup::SharedPtr cb_group_services_;
  rclcpp::CallbackGroup::SharedPtr cb_group_action_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;

  rclcpp::Publisher<rlc_interfaces::msg::JointEffortCmd>::SharedPtr
      effort_pub_;
  rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::
      SharedPtr ctrl_state_pub_;

  rclcpp::Service<rlc_interfaces::srv::SetControllerGains>::SharedPtr
      set_gains_srv_;
  rclcpp::Service<rlc_interfaces::srv::SetControllerMode>::SharedPtr
      set_mode_srv_;

  rclcpp_action::Server<FollowTraj>::SharedPtr traj_action_server_;

  // Preallocated publish messages (control thread only).
  rlc_interfaces::msg::JointEffortCmd effort_cmd_msg_;
  control_msgs::msg::JointTrajectoryControllerState ctrl_state_msg_;
};

}  // namespace rlc_controller_cpp
