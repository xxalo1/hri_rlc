#pragma once

#include <atomic>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <cstddef>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rbt_core_cpp/robot.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "rlc_utils/types.hpp"

namespace rlc_controller_cpp {

/**
 * @brief ros2_control effort controller plugin built on `rbt_core_cpp::Robot`.
 *
 * @details
 * This controller:
 * - Reads per-joint state interfaces: position [rad], velocity [rad/s].
 * - Writes per-joint command interfaces: effort [N·m].
 * - Performs allocation/validation during lifecycle transitions.
 * - Keeps `update()` allocation-free (read state, run model, write effort).
 *
 * @par Thread safety
 * Not thread-safe. External synchronization is required if accessed
 * concurrently.
 */
class RealtimeController final
    : public controller_interface::ControllerInterface {
 public:
  /**
   * @brief Constructs the controller with empty configuration.
   */
  RealtimeController();

  /**
   * @brief Destroys the controller.
   */
  ~RealtimeController() override = default;

  /**
   * @brief Declares ROS parameters and performs lightweight initialization.
   * @return Lifecycle callback result.
   */
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief Declares the command interfaces claimed by this controller.
   * @return Interface configuration for command interfaces.
   */
  controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  /**
   * @brief Declares the state interfaces claimed by this controller.
   * @return Interface configuration for state interfaces.
   */
  controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  /**
   * @brief Configures the controller (non-realtime).
   * @param[in] previous_state Previous lifecycle state.
   * @return Lifecycle callback result.
   */
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Activates the controller and prepares realtime execution.
   * @param[in] previous_state Previous lifecycle state.
   * @return Lifecycle callback result.
   */
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Deactivates the controller and stops realtime execution.
   * @param[in] previous_state Previous lifecycle state.
   * @return Lifecycle callback result.
   */
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Runs one realtime control cycle.
   * @param[in] time Current ROS time stamp.
   * @param[in] period Time since the last update call.
   * @return Controller return code.
   */
  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  /**
   * @brief Raw state interface pointers for one joint.
   *
   * @details
   * These pointers are non-owning and refer to loaned interfaces managed by
   * ros2_control. They are resolved during activation and must remain valid for
   * the controller lifetime while active.
   */
  struct JointStateInterfaces {
    hardware_interface::LoanedStateInterface* position{nullptr};
    hardware_interface::LoanedStateInterface* velocity{nullptr};
  };

  /**
   * @brief Raw command interface pointers for one joint.
   *
   * @details
   * These pointers are non-owning and refer to loaned interfaces managed by
   * ros2_control. They are resolved during activation and must remain valid for
   * the controller lifetime while active.
   */
  struct JointCommandInterfaces {
    hardware_interface::LoanedCommandInterface* effort{nullptr};
  };

  /**
   * @brief Lightweight joint snapshot used for action feedback.
   *
   * @details
   * Sizes: `q`, `qd`, `q_des`, `qd_des` are `size = ndof()`.
   * - `t`: sample time [s].
   * - `q`: measured joint positions [rad] (source joint order).
   * - `qd`: measured joint velocities [rad/s] (source joint order).
   * - `q_des`: desired joint positions [rad] (source joint order).
   * - `qd_des`: desired joint velocities [rad/s] (source joint order).
   * - `active_traj_seq`: identifier of the currently applied trajectory.
   */
  struct JointSnapshot {
    double t{0.0};
    std::vector<double> q;
    std::vector<double> qd;
    std::vector<double> q_des;
    std::vector<double> qd_des;
    uint64_t active_traj_seq{0};

    JointSnapshot() = default;
    explicit JointSnapshot(std::size_t n) { resize(n); }

    /**
     * @brief Resizes all vectors to `n`.
     * @param[in] n Number of joints (DOF).
     */
    void resize(std::size_t n) {
      q.resize(n);
      qd.resize(n);
      q_des.resize(n);
      qd_des.resize(n);
    }
  };

  /**
   * @brief Pending trajectory command transferred to the realtime thread.
   */
  struct PendingJointTrajectory {
    uint64_t seq{0};
    std::shared_ptr<const rbt_core_cpp::JointTrajectory> traj;
  };

  using FollowJTAction = control_msgs::action::FollowJointTrajectory;
  using FollowJTGoalHandle = rclcpp_action::ServerGoalHandle<FollowJTAction>;

  /**
   * @brief Resolves and caches loaned command/state interface handles.
   * @return Lifecycle callback result.
   */
  controller_interface::CallbackReturn init_interface_handles();

  /**
   * @brief Reads state and writes an initial hold effort.
   * @return Lifecycle callback result.
   */
  controller_interface::CallbackReturn calibrate();

  /**
   * @brief Creates and starts the FollowJointTrajectory action server.
   * @return Lifecycle callback result.
   */
  controller_interface::CallbackReturn init_traj_action_server();

  /**
   * @brief Validates and converts an incoming JointTrajectory message.
   * @param[in] jt Joint trajectory message in source joint order.
   * @param[out] msg_data Converted trajectory:
   *   - `t`: time [s], shape = (N,), strictly increasing.
   *   - `q`: joint positions [rad], shape = (N, ndof()).
   *   - `qd`: joint velocities [rad/s], shape = (N, ndof()).
   *   - `qdd`: joint accelerations [rad/s^2], shape = (N, ndof()).
   * @param[out] error_string Optional error description; may be nullptr.
   * @return True on success.
   */
  bool build_traj_command(const trajectory_msgs::msg::JointTrajectory& jt,
                          rlc_utils::types::JointTrajectoryMsgData& msg_data,
                          std::string* error_string);

  /**
   * @brief Action callback to accept or reject a trajectory goal
   * (non-realtime).
   * @param[in] goal_uuid Goal UUID.
   * @param[in] goal Goal request.
   * @return Goal response.
   */
  rclcpp_action::GoalResponse handle_traj_goal(
      const rclcpp_action::GoalUUID& goal_uuid,
      std::shared_ptr<const FollowJTAction::Goal> goal);

  /**
   * @brief Action callback to handle a cancel request (non-realtime).
   * @param[in] goal_handle Goal handle.
   * @return Cancel response.
   */
  rclcpp_action::CancelResponse handle_traj_cancel(
      const std::shared_ptr<FollowJTGoalHandle> goal_handle);

  /**
   * @brief Action callback invoked when a goal is accepted (non-realtime).
   * @param[in] goal_handle Accepted goal handle.
   */
  void handle_traj_accepted(
      const std::shared_ptr<FollowJTGoalHandle> goal_handle);

  /**
   * @brief Worker routine that executes an accepted goal (non-realtime).
   * @param[in] goal_handle Goal handle.
   * @param[in] preempt_flag Preemption flag shared with newer goals.
   */
  void execute_traj_goal(std::shared_ptr<FollowJTGoalHandle> goal_handle,
                         std::shared_ptr<std::atomic_bool> preempt_flag);

  /**
   * @brief Applies any pending trajectory to the model (realtime).
   */
  void consume_pending_traj_rt();

  /**
   * @brief Publishes the latest state/desired snapshot (realtime).
   */
  void update_snapshot_rt();

  /**
   * @brief Reads joint state interfaces into internal buffers (realtime).
   * @param[in] time Current ROS time.
   * @return Controller return code.
   */
  controller_interface::return_type read_state(const rclcpp::Time& time);

  /**
   * @brief Writes effort commands to command interfaces (realtime).
   * @return Controller return code.
   */
  controller_interface::return_type write_cmd();

  /**
   * @brief Returns the configured number of joints.
   * @return Number of joints.
   */
  std::size_t ndof() const { return ndof_; }

  /**
   * @brief Sets the configured number of joints.
   * @param[in] ndof Number of joints.
   */
  void set_ndof(std::size_t ndof) { ndof_ = ndof; }

  std::size_t ndof_{0};
  std::vector<std::string> joints_;
  std::vector<JointStateInterfaces> state_handles_;
  std::vector<JointCommandInterfaces> command_handles_;

  std::optional<rbt_core_cpp::Robot> robot_;

  rlc_utils::types::JointStateMsgData joint_state_;
  rlc_utils::types::JointEffortCmdMsgData joint_cmd_;

  std::vector<std::string> claimed_state_interfaces_;
  std::vector<std::string> claimed_command_interfaces_;

  rclcpp_action::Server<FollowJTAction>::SharedPtr traj_action_server_{};
  std::atomic_bool is_active_{false};

  // Action goal bookkeeping (non-RT).
  std::mutex action_mutex_;
  std::shared_ptr<FollowJTGoalHandle> active_goal_{};
  std::shared_ptr<std::atomic_bool> active_preempt_flag_{};

  // Worker thread management (non-RT).
  std::mutex worker_mutex_;
  std::vector<std::thread> worker_threads_;

  // Stop condition for worker loops.
  std::atomic_bool shutting_down_{false};

  realtime_tools::RealtimeThreadSafeBox<JointSnapshot> snapshot_box_;
  JointSnapshot snapshot_rt_;
  uint64_t last_applied_traj_seq_{0};
  realtime_tools::RealtimeBuffer<std::shared_ptr<PendingJointTrajectory>>
      pending_traj_;
};

}  // namespace rlc_controller_cpp
