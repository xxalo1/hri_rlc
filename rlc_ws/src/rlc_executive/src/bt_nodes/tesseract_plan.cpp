#include "rlc_executive/bt_nodes/tesseract_plan.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <tesseract_monitoring/environment_monitor_interface.h>

#include "rbt_planning/trajopt_planner.hpp"

#include "rlc_executive/bt_nodes/bt_utils.hpp"
#include "rlc_executive/core/runtime_context.hpp"

namespace rlc_executive
{

TesseractPlan::TesseractPlan(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

std::shared_ptr<TesseractPlan::TrajOptPlanner>
TesseractPlan::initializePlanner(const std::string& group_name, double trajectory_dt)
{
  const std::string& monitor_namespace = ctx_->config().monitor.monitor_namespace;
  auto env_uptr = ctx_->envMonitorInterface().getEnvironment(monitor_namespace);
  if (!env_uptr)
  {
    throw BT::RuntimeError(registrationName(), "[", fullPath(),
                           "]: failed to snapshot Tesseract environment from namespace '",
                           monitor_namespace, "'");
  }

  std::shared_ptr<tesseract_environment::Environment> env{ std::move(env_uptr) };

  TrajOptPlanner::Options planner_opt;
  if (const PlanningProfile* profile = ctx_->getDefaultPlanningProfile();
      profile != nullptr)
  {
    planner_opt.ompl_planning_time = profile->allowed_planning_time_sec;
  }
  planner_opt.dt = trajectory_dt;
  planner_opt.use_ompl_seed = true;
  planner_opt.trajopt_num_steps = 20;

  return std::make_shared<TrajOptPlanner>(env, group_name, planner_opt);
}

void TesseractPlan::runPlannerWorker(JointEndpoints joint_endpoints,
                                     RewardTerms reward_terms)
{
  const auto started_at = std::chrono::steady_clock::now();

  try
  {
    auto trajectory = planner_->plan(joint_endpoints, reward_terms);

    const double elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at)
            .count();

    std::scoped_lock lock(result_mutex_);
    trajectory_ = std::make_shared<Trajectory>(std::move(trajectory));
    error_.clear();
    elapsed_sec_ = elapsed_sec;
    worker_done_ = true;
  }
  catch (const std::exception& e)
  {
    const double elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at)
            .count();

    std::scoped_lock lock(result_mutex_);
    trajectory_.reset();
    error_ = e.what();
    elapsed_sec_ = elapsed_sec;
    worker_done_ = true;
  }
}

BT::NodeStatus TesseractPlan::onStart()
{
  try
  {
    bt_utils::ensureContextAndLogger(*this, ctx_, logger_);

    if (worker_thread_.joinable())
    {
      worker_thread_.join();
    }

    const auto joint_endpoints =
        bt_utils::requireInput<JointEndpoints>(*this, PortKeys::JOINT_ENDPOINTS);

    const auto reward_terms =
        bt_utils::requireInput<RewardTerms>(*this, PortKeys::REWARD_TERMS);

    const double trajectory_dt =
        bt_utils::requireInput<double>(*this, PortKeys::TRAJECTORY_DT);
    if (!(trajectory_dt > 0.0))
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(), "]: input port '",
                             PortKeys::TRAJECTORY_DT, "' must be > 0");
    }

    const std::string group_name =
        bt_utils::requireInput<std::string>(*this, PortKeys::GROUP_NAME);

    planner_ = initializePlanner(group_name, trajectory_dt);

    {
      std::scoped_lock lock(result_mutex_);
      trajectory_.reset();
      error_.clear();
      elapsed_sec_ = 0.0;
      worker_done_ = false;
    }
    bt_utils::setOutput(*this, PortKeys::TRAJECTORY, std::shared_ptr<const Trajectory>());

    worker_thread_ = std::jthread(&TesseractPlan::runPlannerWorker, this, joint_endpoints,
                                  reward_terms);

    return BT::NodeStatus::RUNNING;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

BT::NodeStatus TesseractPlan::onRunning()
{
  try
  {
    std::shared_ptr<const Trajectory> trajectory;
    std::string error;
    double elapsed_sec = 0.0;
    bool worker_done = false;
    {
      std::scoped_lock lock(result_mutex_);
      trajectory = trajectory_;
      error = error_;
      elapsed_sec = elapsed_sec_;
      worker_done = worker_done_;
    }

    if (!worker_done)
    {
      return BT::NodeStatus::RUNNING;
    }

    if (worker_thread_.joinable())
    {
      worker_thread_.join();
    }

    if (!error.empty())
    {
      return failWithError(error, elapsed_sec);
    }

    if (!trajectory)
    {
      return failWithError("planner failed to produce a trajectory", elapsed_sec);
    }

    bt_utils::setOutput(*this, PortKeys::TRAJECTORY, trajectory);

    bt_utils::setSuccessDiagnostics(*this, elapsed_sec);
    RCLCPP_INFO(*logger_, "SUCCESS (%.3fs) traj_points=%zu traj_joints=%zu", elapsed_sec,
                trajectory->length(), trajectory->dim());
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

void TesseractPlan::onHalted()
{
  if (logger_)
  {
    RCLCPP_WARN(*logger_, "halt requested, planner will finish in background");
  }
}

BT::NodeStatus TesseractPlan::failWithError(const std::string& msg, double elapsed_sec)
{
  bt_utils::setOutput(*this, PortKeys::TRAJECTORY, std::shared_ptr<const Trajectory>());
  bt_utils::setFailureDiagnostics(*this, msg, elapsed_sec);
  if (logger_)
  {
    RCLCPP_ERROR(*logger_, "%s", msg.c_str());
  }
  return BT::NodeStatus::FAILURE;
}

void registerTesseractPlanNode(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<TesseractPlan>("TesseractPlan");
}

}  // namespace rlc_executive
