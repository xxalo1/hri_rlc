#include <rlc_planner/trajopt_interface.hpp>

#include <rbt_planning/trajopt_solver.hpp>

#include <moveit/robot_state/conversions.hpp>
#include <moveit/utils/logger.hpp>
#include <moveit/utils/moveit_error_code.hpp>

#include <Eigen/Core>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rlc_planner {
namespace {

rclcpp::Logger getLogger() {
  return moveit::getLogger("rlc_planner.trajopt_interface");
}

constexpr char kPlannerId[] = "rlc_trajopt";

bool has_start_state(const moveit_msgs::msg::RobotState& start_state) {
  return !start_state.joint_state.name.empty() ||
         !start_state.multi_dof_joint_state.joint_names.empty() ||
         !start_state.attached_collision_objects.empty() || start_state.is_diff;
}

}  // namespace

TrajOptInterface::TrajOptInterface(const rclcpp::Node::SharedPtr& node,
                                   std::string parameter_namespace,
                                   Options options)
    : node_(node),
      parameter_namespace_(std::move(parameter_namespace)),
      options_(options) {}

bool TrajOptInterface::ensure_initialized(std::string& error) {
  std::scoped_lock<std::mutex> lk(init_mutex_);
  if (env_adapter_initialized_) {
    return true;
  }
  if (!node_) {
    error = "Node is null";
    return false;
  }

  std::string urdf_xml;
  std::string srdf_xml;
  node_->get_parameter_or<std::string>("robot_description", urdf_xml, "");
  node_->get_parameter_or<std::string>("robot_description_semantic", srdf_xml,
                                       "");

  if (urdf_xml.empty()) {
    error = "Missing parameter: robot_description";
    return false;
  }
  if (srdf_xml.empty()) {
    error = "Missing parameter: robot_description_semantic";
    return false;
  }

  try {
    env_adapter_.initialize(urdf_xml, srdf_xml);
  } catch (const std::exception& e) {
    error = std::string{"Failed to initialize Tesseract environment: "} +
            e.what();
    return false;
  }

  env_adapter_initialized_ = true;
  return true;
}

int TrajOptInterface::resolve_num_steps() const {
  if (!node_) {
    return options_.default_num_steps;
  }

  const std::string key =
      parameter_namespace_.empty() ? "num_steps"
                                   : (parameter_namespace_ + ".num_steps");

  int n = options_.default_num_steps;
  node_->get_parameter_or<int>(key, n, options_.default_num_steps);
  return n;
}

bool TrajOptInterface::plan(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    planning_interface::MotionPlanResponse& res) {
  res.trajectory.reset();
  res.planning_time = 0.0;
  res.planner_id = req.planner_id.empty() ? std::string{kPlannerId}
                                          : req.planner_id;

  if (!planning_scene) {
    res.error_code = moveit::core::MoveItErrorCode(
        moveit_msgs::msg::MoveItErrorCodes::FAILURE, "PlanningScene is null",
        "rlc_planner");
    return false;
  }

  std::string init_error;
  if (!ensure_initialized(init_error)) {
    res.error_code = moveit::core::MoveItErrorCode(
        moveit_msgs::msg::MoveItErrorCodes::FAILURE, init_error, "rlc_planner");
    return false;
  }

  if (req.group_name.empty()) {
    res.error_code = moveit::core::MoveItErrorCode(
        moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME,
        "group_name is empty", "rlc_planner");
    return false;
  }

  if (req.goal_constraints.empty()) {
    res.error_code = moveit::core::MoveItErrorCode(
        moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS,
        "goal_constraints is empty (joint goals only supported for now)",
        "rlc_planner");
    return false;
  }

  if (!req.path_constraints.joint_constraints.empty() ||
      !req.path_constraints.position_constraints.empty() ||
      !req.path_constraints.orientation_constraints.empty() ||
      !req.path_constraints.visibility_constraints.empty()) {
    res.error_code = moveit::core::MoveItErrorCode(
        moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN,
        "path_constraints are not supported yet", "rlc_planner");
    return false;
  }

  const auto robot_model = planning_scene->getRobotModel();
  if (!robot_model) {
    res.error_code = moveit::core::MoveItErrorCode(
        moveit_msgs::msg::MoveItErrorCodes::FAILURE, "RobotModel is null",
        "rlc_planner");
    return false;
  }

  const moveit::core::JointModelGroup* jmg =
      robot_model->getJointModelGroup(req.group_name);
  if (!jmg) {
    res.error_code = moveit::core::MoveItErrorCode(
        moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME,
        "Unknown group_name: '" + req.group_name + "'", "rlc_planner");
    return false;
  }

  const std::vector<std::string>& joint_names = jmg->getVariableNames();
  if (joint_names.empty()) {
    res.error_code = moveit::core::MoveItErrorCode(
        moveit_msgs::msg::MoveItErrorCodes::FAILURE,
        "No joint variables found for group '" + req.group_name + "'",
        "rlc_planner");
    return false;
  }

  moveit::core::RobotState start_state(planning_scene->getCurrentState());
  if (has_start_state(req.start_state)) {
    if (!moveit::core::robotStateMsgToRobotState(req.start_state, start_state,
                                                 true)) {
      res.error_code = moveit::core::MoveItErrorCode(
          moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE,
          "Failed to parse req.start_state", "rlc_planner");
      return false;
    }
  }

  Eigen::VectorXd q_start(joint_names.size());
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    q_start[static_cast<long>(i)] =
        start_state.getVariablePosition(joint_names[i]);
  }

  const auto& goal = req.goal_constraints.front();
  if (goal.joint_constraints.empty()) {
    res.error_code = moveit::core::MoveItErrorCode(
        moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS,
        "Only joint goal constraints are supported for now", "rlc_planner");
    return false;
  }

  std::unordered_map<std::string, double> goal_map;
  goal_map.reserve(goal.joint_constraints.size());
  for (const auto& jc : goal.joint_constraints) {
    goal_map.emplace(jc.joint_name, jc.position);
  }

  Eigen::VectorXd q_goal(joint_names.size());
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    const auto it = goal_map.find(joint_names[i]);
    if (it == goal_map.end()) {
      res.error_code = moveit::core::MoveItErrorCode(
          moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS,
          "Missing joint constraint for '" + joint_names[i] +
              "' (goal_constraints[0])",
          "rlc_planner");
      return false;
    }
    q_goal[static_cast<long>(i)] = it->second;
  }

  try {
    auto env_uptr = env_adapter_.make_snapshot(planning_scene);
    auto env = std::shared_ptr<tesseract_environment::Environment>(
        std::move(env_uptr));

    rbt_planning::TrajoptSolver solver(std::move(env));
    solver.set_manipulator_group(req.group_name);
    solver.set_num_steps(resolve_num_steps());
    solver.set_start_state(joint_names, q_start);
    solver.set_goal_state(q_goal);

    const auto t0 = node_->now();
    trajopt::TrajArray traj = solver.solve();
    const auto t1 = node_->now();
    res.planning_time = (t1 - t0).seconds();

    auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(
        robot_model, req.group_name);

    moveit::core::RobotState waypoint_state(start_state);
    for (int t = 0; t < traj.rows(); ++t) {
      for (std::size_t j = 0; j < joint_names.size(); ++j) {
        waypoint_state.setVariablePosition(
            joint_names[j], traj(t, static_cast<int>(j)));
      }
      waypoint_state.update();
      trajectory->addSuffixWaypoint(waypoint_state, 0.0);
    }

    res.trajectory = std::move(trajectory);
    moveit::core::robotStateToRobotStateMsg(start_state, res.start_state, true);
    res.error_code = moveit::core::MoveItErrorCode(
        moveit_msgs::msg::MoveItErrorCodes::SUCCESS, "", "rlc_planner");
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(getLogger(), "TrajOptInterface::plan failed: %s", e.what());
    res.error_code = moveit::core::MoveItErrorCode(
        moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED, e.what(),
        "rlc_planner");
    return false;
  }
}

}  // namespace rlc_planner
