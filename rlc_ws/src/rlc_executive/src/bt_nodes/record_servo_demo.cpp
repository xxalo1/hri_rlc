#include "rlc_executive/bt_nodes/record_servo_demo.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_msgs/msg/servo_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rlc_executive/bt_nodes/bt_utils.hpp"
#include "rlc_executive/core/types.hpp"
#include "rlc_executive/core/runtime_context.hpp"
#include "rlc_executive/servo/teleoperation_controller.hpp"
#include "rlc_executive/state/state_buffer.hpp"

namespace rlc_executive
{
namespace
{

bool isHardServoHalt(std::int8_t code)
{
  return code == moveit_msgs::msg::ServoStatus::INVALID ||
         code == moveit_msgs::msg::ServoStatus::HALT_FOR_SINGULARITY ||
         code == moveit_msgs::msg::ServoStatus::HALT_FOR_COLLISION ||
         code == moveit_msgs::msg::ServoStatus::JOINT_BOUND;
}

std::string makeServoHaltMessage(const moveit_msgs::msg::ServoStatus& status)
{
  if (!status.message.empty())
  {
    return "servo halted: " + status.message;
  }
  return "servo halted with status code " + std::to_string(status.code);
}

}  // namespace

RecordServoDemo::RecordServoDemo(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

void RecordServoDemo::requestWorkerStop() noexcept
{
  if (worker_thread_.joinable())
  {
    worker_thread_.request_stop();
    stop_cv_.notify_all();
  }
}

void RecordServoDemo::joinWorker() noexcept
{
  if (worker_thread_.joinable())
  {
    worker_thread_.join();
  }
}

void RecordServoDemo::requestServoPaused(bool pause) noexcept
{
  if (!ctx_)
  {
    return;
  }

  const ServoPauseResult result = ctx_->teleoperationController().setActive(!pause);
  if (!result.success && logger_)
  {
    RCLCPP_WARN(*logger_, "failed to set servo to %s: %s", pause ? "paused" : "unpaused",
                result.message.c_str());
  }
}

void RecordServoDemo::stopAndJoinWorker() noexcept
{
  if (worker_thread_.joinable())
  {
    worker_thread_.request_stop();
    worker_thread_.join();
  }
}

RecordServoDemo::ProgressState RecordServoDemo::readProgress() const
{
  std::scoped_lock lock(recording_mutex_);
  return progress_;
}

void RecordServoDemo::writeProgress(std::optional<double> elapsed_sec,
                                    std::optional<bool> failed,
                                    std::optional<std::string> feedback_msg,
                                    std::optional<std::string> error_msg,
                                    std::optional<std::size_t> sample_count)
{
  std::scoped_lock lock(recording_mutex_);
  writeProgressLocked(elapsed_sec, failed, std::move(feedback_msg), std::move(error_msg),
                      sample_count);
}

void RecordServoDemo::writeProgressLocked(std::optional<double> elapsed_sec,
                                          std::optional<bool> failed,
                                          std::optional<std::string> feedback_msg,
                                          std::optional<std::string> error_msg,
                                          std::optional<std::size_t> sample_count)
{
  if (elapsed_sec)
  {
    progress_.elapsed_sec = *elapsed_sec;
  }

  if (failed)
  {
    progress_.failed = *failed;
  }

  if (feedback_msg)
  {
    progress_.feedback = std::move(*feedback_msg);
  }

  if (error_msg)
  {
    progress_.error = std::move(*error_msg);
  }

  if (sample_count)
  {
    progress_.sample_count = *sample_count;
  }
}

RecordServoDemo::JointVec
RecordServoDemo::extractPositions(const sensor_msgs::msg::JointState& joint_state) const
{
  const auto joint_dim = static_cast<Eigen::Index>(joint_state.position.size());
  if (joint_dim == 0)
  {
    return JointVec{};
  }
  return Eigen::Map<const JointVec>(joint_state.position.data(), joint_dim);
}

void RecordServoDemo::runSamplingWorker(std::stop_token stop_token)
{
  const auto started_at = std::chrono::steady_clock::now();
  const auto sample_period =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(sample_dt_));
  auto next_sample_time = started_at;

  try
  {
    std::unique_lock<std::mutex> wait_lock(stop_mutex_);

    while (!stop_token.stop_requested())
    {
      wait_lock.unlock();

      const auto joint_state = ctx_->stateBuffer().getLatestJointState();
      if (!joint_state || joint_state->position.size() != joint_dim_)
      {
        const char* const error_msg =
            !joint_state ? "latest joint state is unavailable while recording" :
                           "joint state dimension changed while recording";

        const double elapsed_sec =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at)
                .count();

        writeProgress(elapsed_sec, true, std::string{}, std::string(error_msg),
                      std::nullopt);
        return;
      }

      JointVec sample = extractPositions(*joint_state);
      const double elapsed_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at)
              .count();

      {
        std::scoped_lock lock(recording_mutex_);
        recorded_samples_.push_back(std::move(sample));
        writeProgressLocked(elapsed_sec, std::nullopt, std::nullopt, std::nullopt,
                            recorded_samples_.size());
      }

      wait_lock.lock();
      next_sample_time += sample_period;
      stop_cv_.wait_until(wait_lock, next_sample_time,
                          [&] { return stop_token.stop_requested(); });
    }
  }
  catch (const std::exception& e)
  {
    const double elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at)
            .count();

    writeProgress(elapsed_sec, true, std::string{}, std::string(e.what()), std::nullopt);
  }
}

RecordServoDemo::Trajectory RecordServoDemo::buildRecordedTrajectory() const
{
  std::vector<JointVec> samples;
  {
    std::scoped_lock lock(recording_mutex_);
    samples = recorded_samples_;
  }

  Trajectory trajectory;
  trajectory.dt = sample_dt_;
  trajectory.states.resize(static_cast<Eigen::Index>(samples.size()),
                           static_cast<Eigen::Index>(joint_dim_));

  for (std::size_t row = 0; row < samples.size(); ++row)
  {
    trajectory.states.row(static_cast<Eigen::Index>(row)) = samples[row].transpose();
  }

  trajectory.validate();
  return trajectory;
}

BT::NodeStatus RecordServoDemo::onStart()
{
  try
  {
    bt_utils::ensureContextAndLogger(*this, ctx_, logger_);

    stopAndJoinWorker();

    sample_dt_ = bt_utils::requireInput<double>(*this, PortKeys::SAMPLE_DT);

    if (!ctx_->stateBuffer().hasJointState() || !ctx_->stateBuffer().isJointStateFresh())
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: latest joint state is unavailable or stale");
    }

    const auto joint_state = ctx_->stateBuffer().getLatestJointState();
    if (!joint_state || joint_state->position.empty())
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: latest joint state has no positions");
    }

    {
      std::scoped_lock lock(recording_mutex_);
      joint_dim_ = joint_state->position.size();
      progress_ = ProgressState{};
      writeProgressLocked(0.0, false, std::string("recording servo demo"), std::string{},
                          std::size_t{ 0 });
      recorded_samples_.clear();
    }

    requestServoPaused(false);

    bt_utils::setRunningDiagnostics(*this, "recording servo demo", 0.0);
    worker_thread_ = std::jthread(&RecordServoDemo::runSamplingWorker, this);
    return BT::NodeStatus::RUNNING;
  }
  catch (const std::exception& e)
  {
    if (ctx_)
    {
      const ServoPauseResult pause_result = ctx_->teleoperationController().setActive(false);
      if (!pause_result.success && logger_)
      {
        RCLCPP_WARN(*logger_, "failed to pause servo during startup cleanup: %s",
                    pause_result.message.c_str());
      }
    }
    return failWithError(e.what());
  }
}

BT::NodeStatus RecordServoDemo::onRunning()
{
  try
  {
    ProgressState snapshot = readProgress();

    const std::string feedback =
        snapshot.feedback.empty() ? "recording servo demo" : snapshot.feedback;
    bt_utils::setRunningDiagnostics(*this, feedback, snapshot.elapsed_sec);

    if (snapshot.failed)
    {
      joinWorker();
      requestServoPaused(true);

      return failWithError(snapshot.error.empty() ? "servo demo recording failed" :
                                                    snapshot.error,
                           snapshot.elapsed_sec);
    }

    if (const auto servo_status = ctx_->teleoperationController().status();
        servo_status && isHardServoHalt(servo_status->status.code))
    {
      stopAndJoinWorker();
      requestServoPaused(true);
      return failWithError(makeServoHaltMessage(servo_status->status),
                           snapshot.elapsed_sec);
    }

    const auto request = ctx_->teleoperationController().takeRequest();

    if (request == DemoRequest::NONE)
    {
      return BT::NodeStatus::RUNNING;
    }

    stopAndJoinWorker();
    snapshot = readProgress();

    if (snapshot.failed)
    {
      requestServoPaused(true);
      return failWithError(snapshot.error.empty() ? "servo demo recording failed" :
                                                    snapshot.error,
                           snapshot.elapsed_sec);
    }

    requestServoPaused(true);

    if (request == DemoRequest::ABORT)
    {
      return failWithError("servo demo recording aborted by operator",
                           snapshot.elapsed_sec);
    }

    const std::shared_ptr<const Trajectory> demonstration =
        std::make_shared<Trajectory>(buildRecordedTrajectory());
    bt_utils::setOutput(*this, PortKeys::DEMONSTRATION, demonstration);
    bt_utils::setSuccessDiagnostics(*this, snapshot.elapsed_sec);

    RCLCPP_INFO(*logger_, "SUCCESS (%.3fs, samples=%zu, dof=%zu)", snapshot.elapsed_sec,
                snapshot.sample_count, joint_dim_);
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

void RecordServoDemo::onHalted()
{
  requestWorkerStop();
  requestServoPaused(true);
  joinWorker();

  writeProgress(std::nullopt, std::nullopt, std::string("recording halted"),
                std::string{}, std::nullopt);

  if (logger_)
  {
    RCLCPP_WARN(*logger_, "halt requested, servo demo recording stopped");
  }
}

BT::NodeStatus RecordServoDemo::failWithError(const std::string& msg, double elapsed_sec)
{
  bt_utils::setFailureDiagnostics(*this, msg, elapsed_sec);
  if (logger_)
  {
    RCLCPP_ERROR(*logger_, "%s", msg.c_str());
  }
  return BT::NodeStatus::FAILURE;
}

void registerRecordServoDemoNode(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<RecordServoDemo>("RecordServoDemo");
}

}  // namespace rlc_executive
