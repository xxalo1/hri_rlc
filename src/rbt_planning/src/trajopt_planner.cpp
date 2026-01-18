#include "rbt_planning/trajopt_planner.hpp"

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_planning/core/utils.h>
#include <tesseract_planning/process_planners/process_planner_names.h>
#include <tesseract_process_managers/core/process_planning_server.h>

#include <stdexcept>

namespace rbt_planning {

static void validateRequest(const TrajoptRequest& req) {
  if (req.joint_names.empty())
    throw std::runtime_error("TrajoptRequest.joint_names is empty");

  const auto n = static_cast<long>(req.joint_names.size());
  if (req.q_start.size() != n || req.q_goal.size() != n)
    throw std::runtime_error(
        "q_start/q_goal dimension does not match joint_names size");
}

TrajoptPlanner::TrajoptPlanner(
    std::shared_ptr<const tesseract_environment::Environment> env)
    : env_(std::move(env)) {
  if (!env_) throw std::runtime_error("TrajoptPlanner: env is null");
}

tesseract_common::JointTrajectory TrajoptPlanner::plan(
    const TrajoptRequest& req) const {
  validateRequest(req);

  using tesseract_planning::CompositeInstruction;
  using tesseract_planning::CompositeInstructionOrder;
  using tesseract_planning::Instruction;
  using tesseract_planning::ManipulatorInfo;
  using tesseract_planning::PlanInstruction;
  using tesseract_planning::PlanInstructionType;
  using tesseract_planning::ProcessPlanningFuture;
  using tesseract_planning::ProcessPlanningRequest;
  using tesseract_planning::ProcessPlanningServer;
  using tesseract_planning::StateWaypoint;
  using tesseract_planning::Waypoint;

  auto env = env_->clone();
  env->setState(req.joint_names, req.q_start);

  CompositeInstruction program("rbt_freespace",
                               CompositeInstructionOrder::ORDERED,
                               ManipulatorInfo(req.manipulator));

  Waypoint start_wp = StateWaypoint(req.joint_names, req.q_start);
  PlanInstruction start_instruction(start_wp, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  Waypoint goal_wp = StateWaypoint(req.joint_names, req.q_goal);
  PlanInstruction plan_to_goal(goal_wp, PlanInstructionType::FREESPACE,
                               req.profile);
  program.push_back(plan_to_goal);

  ProcessPlanningServer planning_server(
      std::make_shared<tesseract_environment::DefaultEnvironmentCache>(env), 1);
  planning_server.loadDefaultProcessPlanners();

  ProcessPlanningRequest request;
  request.name =
      tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  request.instructions = Instruction(program);

  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  if (!response.results)
    throw std::runtime_error("TrajOpt pipeline returned null results");

  const auto* ci =
      response.results->cast_const<tesseract_planning::CompositeInstruction>();
  if (ci == nullptr)
    throw std::runtime_error(
        "TrajOpt pipeline results were not a CompositeInstruction");

  return tesseract_planning::toJointTrajectory(*ci);
}

}  // namespace rbt_planning
