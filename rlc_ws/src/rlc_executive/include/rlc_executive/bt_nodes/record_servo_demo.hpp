#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rbt_types/math.hpp"
#include "rbt_types/trajectory.hpp"

#include "rlc_executive/bt_nodes/bt_utils.hpp"

namespace rlc_executive
{

class RuntimeContext;

/**
 * @brief Records a servo-driven joint-space demonstration into one trajectory.
 *
 * @details
 * This BT node unpauses MoveIt Servo on start, samples the latest joint state from the
 * runtime `StateBuffer` at a fixed sample period, and publishes the recorded
 * trajectory to the `demonstration` port when `stop_recording` becomes true.
 * Joint positions are recorded exactly as they arrive in `JointState.position`,
 * so the saved trajectory uses the incoming joint ordering and current joint count
 * without filtering or remapping.
 *
 * Input ports:
 * - `sample_dt`: Fixed sample period [s] used for the saved trajectory.
 * - `stop_recording`: Level-triggered stop flag. It should be `false` when recording
 *   starts and set to `true` when the demonstration should finish.
 *
 * Output ports:
 * - `demonstration`: Recorded joint-space trajectory [rad], shape =
 *   (num_samples, joint_dim), RowMajor.
 * - `error`: Human-readable failure text.
 * - `feedback`: Running progress text.
 * - `elapsed_sec`: Elapsed recording time [s].
 *
 * The node fails if Servo reports a hard halt, joint states become unavailable/stale,
 * or the recorded demo can not be finalized into a valid fixed-step trajectory.
 */
class RecordServoDemo final : public BT::StatefulActionNode
{
public:
  using Trajectory = rbt_types::Trajectory;
  using JointVec = rbt_types::JointVec;

  /**
   * @brief Constructs the servo demo recorder BT node.
   * @param[in] name Node instance name used by BehaviorTree.CPP.
   * @param[in] config BehaviorTree node configuration, including blackboard access.
   */
  RecordServoDemo(const std::string& name, const BT::NodeConfig& config);

  /**
   * @brief Declares the ports consumed and produced by this BT node.
   * @return BehaviorTree port list containing recording inputs, the recorded
   * demonstration output, and standard diagnostic outputs.
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(PortKeys::SAMPLE_DT, 0.05, ""),
      BT::InputPort<bool>(PortKeys::STOP_RECORDING, false, ""),
      BT::OutputPort<std::shared_ptr<const Trajectory>>(PortKeys::DEMONSTRATION),
      BT::OutputPort<std::string>(PortKeys::ERROR),
      BT::OutputPort<std::string>(PortKeys::FEEDBACK),
      BT::OutputPort<double>(PortKeys::ELAPSED_SEC),
    };
  }

  /**
   * @brief Blackboard key names used by RecordServoDemo.
   */
  struct PortKeys : bt_utils::DiagnosticPortKeys
  {
    static inline const std::string SAMPLE_DT = "sample_dt";
    static inline const std::string STOP_RECORDING = "stop_recording";
    static inline const std::string DEMONSTRATION = "demonstration";
  };

private:
  struct ProgressState
  {
    std::string feedback;
    std::string error;
    double elapsed_sec{ 0.0 };
    std::size_t sample_count{ 0 };
    bool failed{ false };
  };

  JointVec extractPositions(const sensor_msgs::msg::JointState& joint_state) const;
  void runSamplingWorker(std::stop_token stop_token);
  Trajectory buildRecordedTrajectory() const;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  BT::NodeStatus failWithError(const std::string& msg, double elapsed_sec = 0.0);

  void requestWorkerStop() noexcept;
  void joinWorker() noexcept;
  void requestServoPaused(bool pause) noexcept;
  void stopAndJoinWorker() noexcept;

  ProgressState readProgress() const;
  void writeProgress(std::optional<double> elapsed_sec = std::nullopt,
                     std::optional<bool> failed = std::nullopt,
                     std::optional<std::string> feedback_msg = std::nullopt,
                     std::optional<std::string> error_msg = std::nullopt,
                     std::optional<std::size_t> sample_count = std::nullopt);
  void writeProgressLocked(std::optional<double> elapsed_sec, std::optional<bool> failed,
                           std::optional<std::string> feedback_msg,
                           std::optional<std::string> error_msg,
                           std::optional<std::size_t> sample_count);

  std::shared_ptr<RuntimeContext> ctx_;
  std::optional<rclcpp::Logger> logger_;
  double sample_dt_{ 0.05 };
  std::size_t joint_dim_{ 0 };
  mutable std::mutex recording_mutex_;
  ProgressState progress_{};
  std::vector<JointVec> recorded_samples_;

  std::jthread worker_thread_{};
  std::mutex stop_mutex_;
  std::condition_variable stop_cv_;
};

}  // namespace rlc_executive
