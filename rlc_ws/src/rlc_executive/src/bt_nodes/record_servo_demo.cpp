#include "rlc_executive/bt_nodes/record_servo_demo.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_msgs/msg/servo_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor_interface.h>

#include "rlc_executive/bt_nodes/bt_utils.hpp"
#include "rlc_executive/core/runtime_context.hpp"
#include "rlc_executive/moveit/moveit_servo_client.hpp"
#include "rlc_executive/state/state_buffer.hpp"

namespace rlc_executive
{
namespace
{

std::string makeRecordingFeedback(std::size_t sample_count)
{
  return "recording servo demo: " + std::to_string(sample_count) + " samples";
}

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

std::vector<std::string>
RecordServoDemo::resolveGroupJointNames(const std::string& group_name) const
{
  const std::string& monitor_namespace = ctx_->config().monitor.monitor_namespace;
  auto env = ctx_->envMonitorInterface().getEnvironment(monitor_namespace);
  if (!env)
  {
    throw BT::RuntimeError(registrationName(), "[", fullPath(),
                           "]: failed to snapshot Tesseract environment from namespace '",
                           monitor_namespace, "'");
  }

  try
  {
    std::vector<std::string> joint_names = env->getGroupJointNames(group_name);
    if (joint_names.empty())
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: manipulator group '", group_name,
                             "' has no active joints");
    }
    return joint_names;
  }
  catch (const std::exception& e)
  {
    throw BT::RuntimeError(registrationName(), "[", fullPath(),
                           "]: failed to resolve joints for manipulator group '",
                           group_name, "': ", e.what());
  }
}

RecordServoDemo::JointVec
RecordServoDemo::extractGroupPositions(const sensor_msgs::msg::JointState& joint_state) const
{
  if (joint_state.name.size() != joint_state.position.size())
  {
    throw std::runtime_error(
        "joint state name/position arrays have different lengths");
  }

  JointVec positions(static_cast<Eigen::Index>(joint_names_.size()));

  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    const auto it =
        std::find(joint_state.name.begin(), joint_state.name.end(), joint_names_[i]);
    if (it == joint_state.name.end())
    {
      throw std::runtime_error("joint state is missing required joint '" +
                               joint_names_[i] + "'");
    }

    const auto index = static_cast<std::size_t>(
        std::distance(joint_state.name.begin(), it));
    positions[static_cast<Eigen::Index>(i)] = joint_state.position[index];
  }

  return positions;
}

void RecordServoDemo::runSamplingWorker(std::stop_token stop_token)
{
  const auto started_at = std::chrono::steady_clock::now();
  const auto sample_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(sample_dt_));
  auto next_sample_time = started_at;

  try
  {
    while (!stop_token.stop_requested())
    {
      const auto joint_state = ctx_->stateBuffer().getLatestJointState();
      if (!joint_state)
      {
        throw std::runtime_error("latest joint state is unavailable while recording");
      }

      JointVec sample = extractGroupPositions(*joint_state);
      const double elapsed_sec = std::chrono::duration<double>(
                                     std::chrono::steady_clock::now() - started_at)
                                     .count();

      {
        std::scoped_lock lock(recording_mutex_);
        recorded_samples_.push_back(std::move(sample));
        progress_.elapsed_sec = elapsed_sec;
        progress_.sample_count = recorded_samples_.size();
        progress_.feedback = makeRecordingFeedback(progress_.sample_count);
      }

      next_sample_time += sample_period;
      std::this_thread::sleep_until(next_sample_time);
    }
  }
  catch (const std::exception& e)
  {
    const double elapsed_sec = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - started_at)
                                   .count();

    std::scoped_lock lock(recording_mutex_);
    progress_.elapsed_sec = elapsed_sec;
    progress_.failed = true;
    progress_.feedback.clear();
    progress_.error = e.what();
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
                           static_cast<Eigen::Index>(joint_names_.size()));

  for (std::size_t row = 0; row < samples.size(); ++row)
  {
    trajectory.states.row(static_cast<Eigen::Index>(row)) = samples[row].transpose();
  }

  trajectory.validate();
  return trajectory;
}

std::shared_ptr<const RecordServoDemo::TrajectorySet>
RecordServoDemo::buildUpdatedDemoSet(const Trajectory& trajectory) const
{
  const auto existing_demos =
      bt_utils::requireInput<std::shared_ptr<const TrajectorySet>>(
          *this, PortKeys::DEMONSTRATIONS);

  auto demos = std::make_shared<TrajectorySet>();
  if (existing_demos)
  {
    *demos = *existing_demos;
  }
  demos->push_back(trajectory);
  return demos;
}

BT::NodeStatus RecordServoDemo::onStart()
{
  try
  {
    bt_utils::ensureContextAndLogger(*this, ctx_, logger_);

    if (worker_thread_.joinable())
    {
      worker_thread_.request_stop();
      worker_thread_.join();
    }

    group_name_ = bt_utils::requireInput<std::string>(*this, PortKeys::GROUP_NAME);
    sample_dt_ = bt_utils::requireInput<double>(*this, PortKeys::SAMPLE_DT);

    joint_names_ = resolveGroupJointNames(group_name_);

    if (!ctx_->stateBuffer().hasJointState() || !ctx_->stateBuffer().isJointStateFresh())
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: latest joint state is unavailable or stale");
    }

    {
      std::scoped_lock lock(recording_mutex_);
      progress_ = ProgressState{};
      progress_.feedback = "recording servo demo";
      recorded_samples_.clear();
    }

    const ServoPauseResult unpause_result = ctx_->moveItServoClient().setPaused(false);
    if (!unpause_result.success)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: failed to unpause servo: ", unpause_result.message);
    }

    bt_utils::setRunningDiagnostics(*this, "recording servo demo", 0.0);
    worker_thread_ = std::jthread(&RecordServoDemo::runSamplingWorker, this);
    return BT::NodeStatus::RUNNING;
  }
  catch (const std::exception& e)
  {
    if (ctx_)
    {
      const ServoPauseResult pause_result = ctx_->moveItServoClient().setPaused(true);
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
    ProgressState snapshot;
    {
      std::scoped_lock lock(recording_mutex_);
      snapshot = progress_;
    }

    const std::string feedback =
        snapshot.feedback.empty() ? "recording servo demo" : snapshot.feedback;
    bt_utils::setRunningDiagnostics(*this, feedback, snapshot.elapsed_sec);

    if (snapshot.failed)
    {
      if (worker_thread_.joinable())
      {
        worker_thread_.join();
      }

      const ServoPauseResult pause_result = ctx_->moveItServoClient().setPaused(true);
      if (!pause_result.success && logger_)
      {
        RCLCPP_WARN(*logger_, "failed to pause servo after recorder failure: %s",
                    pause_result.message.c_str());
      }

      return failWithError(snapshot.error.empty() ? "servo demo recording failed" :
                                                    snapshot.error,
                           snapshot.elapsed_sec);
    }

    if (const auto servo_status = ctx_->moveItServoClient().status();
        servo_status && isHardServoHalt(servo_status->status.code))
    {
      if (worker_thread_.joinable())
      {
        worker_thread_.request_stop();
        worker_thread_.join();
      }

      const ServoPauseResult pause_result = ctx_->moveItServoClient().setPaused(true);
      if (!pause_result.success && logger_)
      {
        RCLCPP_WARN(*logger_, "failed to pause servo after hard halt: %s",
                    pause_result.message.c_str());
      }

      return failWithError(makeServoHaltMessage(servo_status->status),
                           snapshot.elapsed_sec);
    }

    const bool stop_recording =
        bt_utils::requireInput<bool>(*this, PortKeys::STOP_RECORDING);
    if (!stop_recording)
    {
      return BT::NodeStatus::RUNNING;
    }

    if (worker_thread_.joinable())
    {
      worker_thread_.request_stop();
      worker_thread_.join();
    }

    {
      std::scoped_lock lock(recording_mutex_);
      snapshot = progress_;
    }

    if (snapshot.failed)
    {
      const ServoPauseResult pause_result = ctx_->moveItServoClient().setPaused(true);
      if (!pause_result.success && logger_)
      {
        RCLCPP_WARN(*logger_, "failed to pause servo after recorder failure: %s",
                    pause_result.message.c_str());
      }

      return failWithError(snapshot.error.empty() ? "servo demo recording failed" :
                                                    snapshot.error,
                           snapshot.elapsed_sec);
    }

    const ServoPauseResult pause_result = ctx_->moveItServoClient().setPaused(true);
    if (!pause_result.success)
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(),
                             "]: failed to pause servo: ", pause_result.message);
    }

    const Trajectory trajectory = buildRecordedTrajectory();
    const auto demonstrations = buildUpdatedDemoSet(trajectory);
    bt_utils::setOutput(*this, PortKeys::DEMONSTRATIONS, demonstrations);
    bt_utils::setSuccessDiagnostics(*this, snapshot.elapsed_sec);

    RCLCPP_INFO(*logger_, "SUCCESS (%.3fs, samples=%zu, demos=%zu)",
                snapshot.elapsed_sec, snapshot.sample_count,
                demonstrations ? demonstrations->size() : 0U);
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

void RecordServoDemo::onHalted()
{
  if (worker_thread_.joinable())
  {
    worker_thread_.request_stop();
  }

  if (ctx_)
  {
    const ServoPauseResult pause_result = ctx_->moveItServoClient().setPaused(true);
    if (!pause_result.success && logger_)
    {
      RCLCPP_WARN(*logger_, "failed to pause servo during halt: %s",
                  pause_result.message.c_str());
    }
  }

  if (logger_)
  {
    RCLCPP_WARN(*logger_, "halt requested, stopping servo demo recording");
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
