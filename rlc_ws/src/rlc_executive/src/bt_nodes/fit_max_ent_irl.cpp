#include "rlc_executive/bt_nodes/fit_max_ent_irl.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <tesseract_monitoring/environment_monitor_interface.h>

#include "rbt_planning/trajopt_planner.hpp"

#include "rlc_executive/bt_nodes/bt_utils.hpp"
#include "rlc_executive/core/runtime_context.hpp"

namespace rlc_executive
{
namespace
{
FitMaxEntIRL::RewardTerms makeRewardTerms(const FitMaxEntIRL::Features& features,
                                          const FitMaxEntIRL::WeightVec& theta)
{
  FitMaxEntIRL::RewardTerms reward_terms;
  reward_terms.reserve(features.size());

  for (std::size_t i = 0; i < features.size(); ++i)
  {
    reward_terms.push_back({ features[i], theta[i] });
  }

  return reward_terms;
}

std::string makeIterationFeedback(int iteration, double step_norm)
{
  std::ostringstream stream;
  stream << "fitting max-ent irl: iteration " << (iteration + 1)
         << " (step_norm=" << step_norm << ")";
  return stream.str();
}

}  // namespace

FitMaxEntIRL::FitMaxEntIRL(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

rbt_irl::MaxEntIRL::SamplerFn
FitMaxEntIRL::makeTrajectorySampler(const std::string& group_name, double trajectory_dt)
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

  auto planner = std::make_shared<TrajOptPlanner>(env, group_name, planner_opt);

  MaxEntIRL::SamplerFn sampler =
      [planner](const rbt_types::JointEndpointsSet& endpoints_set,
                const rbt_types::RewardTerms& rewards) -> rbt_types::TrajectorySet {
    return planner->plan(endpoints_set, rewards);
  };
  return sampler;
}

void FitMaxEntIRL::initializeIRL(const Features& features, const std::string& group_name,
                                 double trajectory_dt)
{
  MaxEntIRL::SamplerFn sampler = makeTrajectorySampler(group_name, trajectory_dt);
  MaxEntIRL::Options irl_opt;
  irl_opt.learning_rate = 0.99;
  irl_opt.convergence_tol = 0.01;
  irl_opt.max_iters = 100;
  irl_.emplace(features, sampler, irl_opt);
}

void FitMaxEntIRL::iterationCallback(const WeightVec& theta, const WeightVec& theta_prev,
                                     std::chrono::milliseconds elapsed_time,
                                     int iteration)
{
  const double step_norm = (theta - theta_prev).norm();
  const std::string feedback = makeIterationFeedback(iteration, step_norm);
  const double elapsed_sec = std::chrono::duration<double>(elapsed_time).count();

  std::scoped_lock lock(progress_mutex_);
  progress_.feedback = feedback;
  progress_.elapsed_sec = elapsed_sec;
  progress_.iteration = iteration + 1;
}

void FitMaxEntIRL::runFitWorker(std::stop_token stop_token,
                                std::shared_ptr<const TrajectorySet> demo_trajs)
{
  const auto started_at = std::chrono::steady_clock::now();

  try
  {
    irl_->fit(*demo_trajs, {}, stop_token,
              [this](const WeightVec& theta, const WeightVec& theta_prev,
                     std::chrono::milliseconds elapsed_time, int iteration) {
                iterationCallback(theta, theta_prev, elapsed_time, iteration);
              });

    const bool was_canceled = stop_token.stop_requested();
    const double elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at)
            .count();

    std::scoped_lock lock(progress_mutex_);
    progress_.elapsed_sec = elapsed_sec;
    progress_.completed = true;
    progress_.succeeded = !was_canceled;
    progress_.feedback =
        was_canceled ? "max-ent irl fit canceled" : "max-ent irl fit complete";
    progress_.error = was_canceled ? "max-ent irl fit canceled" : "";
  }
  catch (const std::exception& e)
  {
    const double elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at)
            .count();

    std::scoped_lock lock(progress_mutex_);
    progress_.elapsed_sec = elapsed_sec;
    progress_.completed = true;
    progress_.succeeded = false;
    progress_.feedback.clear();
    progress_.error = e.what();
  }
}

BT::NodeStatus FitMaxEntIRL::onStart()
{
  try
  {
    bt_utils::ensureContextAndLogger(*this, ctx_, logger_);

    if (worker_thread_.joinable())
    {
      worker_thread_.request_stop();
      worker_thread_.join();
    }

    bt_utils::setOutput(*this, PortKeys::REWARD_TERMS, RewardTerms{});

    const auto demo_trajs = bt_utils::requireInput<std::shared_ptr<const TrajectorySet>>(
        *this, PortKeys::DEMONSTRATIONS);
    if (!demo_trajs || demo_trajs->empty())
    {
      throw BT::RuntimeError(registrationName(), "[", fullPath(), "]: input port '",
                             PortKeys::DEMONSTRATIONS, "' is null or empty");
    }

    const double dt = demo_trajs->at(0).dt;

    features_ = bt_utils::requireInput<Features>(*this, PortKeys::FEATURES);
    const std::string group_name =
        bt_utils::requireInput<std::string>(*this, PortKeys::GROUP_NAME);

    initializeIRL(features_, group_name, dt);

    {
      std::scoped_lock lock(progress_mutex_);
      progress_ = ProgressState{};
      progress_.feedback = "starting max-ent irl fit";
    }

    bt_utils::setRunningDiagnostics(*this, "starting max-ent irl fit", 0.0);

    worker_thread_ = std::jthread(&FitMaxEntIRL::runFitWorker, this, demo_trajs);

    return BT::NodeStatus::RUNNING;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

BT::NodeStatus FitMaxEntIRL::onRunning()
{
  try
  {
    ProgressState snapshot;
    {
      std::scoped_lock lock(progress_mutex_);
      snapshot = progress_;
    }

    const std::string feedback =
        snapshot.feedback.empty() ? "fitting max-ent irl" : snapshot.feedback;
    bt_utils::setRunningDiagnostics(*this, feedback, snapshot.elapsed_sec);

    if (!snapshot.completed)
    {
      return BT::NodeStatus::RUNNING;
    }

    if (worker_thread_.joinable())
    {
      worker_thread_.join();
    }

    if (!snapshot.succeeded)
    {
      return failWithError(snapshot.error.empty() ? "max-ent irl fit failed" :
                                                    snapshot.error,
                           snapshot.elapsed_sec);
    }

    bt_utils::setOutput(*this, PortKeys::REWARD_TERMS,
                        makeRewardTerms(features_, irl_->theta()));

    bt_utils::setSuccessDiagnostics(*this, snapshot.elapsed_sec);
    RCLCPP_INFO(*logger_, "SUCCESS (%.3fs, iterations=%d)", snapshot.elapsed_sec,
                snapshot.iteration);
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    return failWithError(e.what());
  }
}

void FitMaxEntIRL::onHalted()
{
  if (worker_thread_.joinable())
  {
    worker_thread_.request_stop();
  }

  if (logger_)
  {
    RCLCPP_WARN(*logger_, "halt requested, canceling");
  }
}

BT::NodeStatus FitMaxEntIRL::failWithError(const std::string& msg, double elapsed_sec)
{
  bt_utils::setOutput(*this, PortKeys::REWARD_TERMS, RewardTerms{});
  bt_utils::setFailureDiagnostics(*this, msg, elapsed_sec);
  if (logger_)
  {
    RCLCPP_ERROR(*logger_, "%s", msg.c_str());
  }
  return BT::NodeStatus::FAILURE;
}

void registerFitMaxEntIRLNode(BT::BehaviorTreeFactory& factory)
{ factory.registerNodeType<FitMaxEntIRL>("FitMaxEntIRL"); }

}  // namespace rlc_executive
