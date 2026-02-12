#pragma once

#include <memory>
#include <moveit/planning_interface/planning_interface.hpp>
#include <tesseract_environment/fwd.h>

#include <string>

namespace rlc_planner
{

class TrajOptInterface;

class TrajOptPlanningContext final : public planning_interface::PlanningContext
{
public:
  using EnvironmentPtr = std::shared_ptr<tesseract_environment::Environment>;
  using TrajOptInterfacePtr = std::shared_ptr<TrajOptInterface>;

  TrajOptPlanningContext(const std::string& name, const std::string& group,
                         TrajOptInterfacePtr trajopt_interface,
                         EnvironmentPtr env_snapshot);

  ~TrajOptPlanningContext() override;
  void solve(planning_interface::MotionPlanResponse& res) override;

  void solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;

  void clear() override;

private:
  TrajOptInterfacePtr trajopt_interface_;
  EnvironmentPtr env_snapshot_;
};

}  // namespace rlc_planner
