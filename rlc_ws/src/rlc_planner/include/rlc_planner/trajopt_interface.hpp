#pragma once

/**
 * @file trajopt_interface.hpp
 * @brief MoveIt-to-TrajOpt interface for joint-space motion planning requests.
 */

#include <rbt_planning/trajopt_solver.hpp>
#include <rlc_planner/trajopt_planner_options.hpp>

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/robot_state.hpp>

#include <rclcpp/node.hpp>

#include <tesseract_environment/environment.h>

#include <trajopt/utils.hpp>

#include <Eigen/Core>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace rlc_planner
{

/**
 * @brief Translates a MoveIt planning request into a TrajOpt solve and converts the
 * result back to MoveIt.
 *
 * @details
 * This interface extracts a joint-space start state from the planning scene and a
 * joint-only goal from the request, constructs a TrajOpt problem via
 * `rbt_planning::TrajOptSolver`, and returns a `robot_trajectory::RobotTrajectory`.
 *
 * Current limitations (MVP):
 * - Only 1-DOF active joints are supported in the MoveIt group.
 * - Only `goal_constraints[*].joint_constraints` goals are supported (no pose/visibility
 * constraints).
 * - If `req.trajectory_constraints.constraints` is non-empty, it is interpreted as a
 * joint-space seed trajectory (planner chaining). Each waypoint must provide exactly one
 * `JointConstraint` for each active MoveIt joint, and must not include non-joint
 * constraints.
 * - The MoveIt active joint list must exactly match the configured Tesseract manipulator
 * group joint list (same names and order).
 *
 * Feature costs can be injected via setFeatureCosts() and are applied on every solve.
 * Options are fixed at construction time and returned by options().
 *
 * @par Thread safety
 * Thread-safe with respect to internal configuration and caches (synchronized with
 * mutexes). The provided Tesseract environment snapshot is modified during planning;
 * callers must ensure the same environment instance is not used concurrently by other
 * threads.
 */
class TrajOptInterface final
{
public:
  /**
   * @brief Planning result produced by plan().
   *
   * @details
   * - `trajectory` is a MoveIt `RobotTrajectory` for the requested MoveIt group
   * containing joint positions [rad].
   * - `start_state_msg` is the start state actually used after applying `req.start_state`
   * to the scene.
   *
   * @note The returned trajectory currently uses `waypoint_dt = 0.0` for all waypoints
   * (no timing).
   */
  struct PlanResult
  {
    robot_trajectory::RobotTrajectoryPtr trajectory;  ///< Planned joint-space trajectory.
    moveit_msgs::msg::RobotState
        start_state_msg;  ///< Start state message used for planning.
  };

  /**
   * @brief Constructs a TrajOpt interface with fixed, request-invariant options.
   * @param[in] opt TrajOpt planner options (moved into this instance).
   * @note Invalid option values may be clamped to safe defaults (e.g., `default_num_steps >= 2`).
   */
  explicit TrajOptInterface(TrajOptPlannerOptions::TrajOptOptions opt);

  /**
   * @brief Plans a joint-space trajectory using TrajOpt.
   * @param[in] planning_scene Planning scene used to resolve the robot model and start
   * state.
   * @param[in] req MoveIt motion plan request (uses `group_name`, `start_state`, and
   * joint goal constraints).
   * @param[in,out] env_snapshot Tesseract environment snapshot used for collision
   * checking and kinematics; must not be nullptr. The environment state is updated to the
   * start state during planning.
   * @return Planning result including a MoveIt trajectory and the used start state
   * message.
   * @throws rlc_planner::PlanningError If the request is invalid/unsupported, inputs
   * violate bounds, the seed trajectory is invalid, the joint order does not match
   * between MoveIt and Tesseract, the TrajOpt solve fails, or the resulting trajectory
   * fails MoveIt validity checks.
   * @pre `req.group_name` exists in the scene robot model.
   * @pre `env_snapshot != nullptr`.
   * @pre MoveIt active joints for `req.group_name` exactly match the Tesseract joint list
   * for the selected manipulator group (same names and order).
   */
  PlanResult
  plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
       const planning_interface::MotionPlanRequest& req,
       const std::shared_ptr<tesseract_environment::Environment>& env_snapshot) const;

  /**
   * @brief Replaces the set of user-defined TrajOpt feature costs applied on each solve.
   * @param[in] costs Feature costs to apply (stored internally by value).
   * @note Thread-safe. Subsequent plan() calls use the updated costs.
   */
  void setFeatureCosts(std::vector<rbt_planning::FeatureCost> costs);

  /**
   * @brief Returns the request-invariant TrajOpt options.
   * @return Planner options (reference valid for the lifetime of this interface).
   */
  const TrajOptPlannerOptions::TrajOptOptions& options() const
  {
    return options_;
  }

private:
  struct SeedCacheEntry
  {
    std::vector<std::string> joint_names;
    trajopt::TrajArray seed;
  };

  struct SolutionCacheEntry
  {
    std::vector<std::string> joint_names;
    trajopt::TrajArray traj;
  };

  struct StartData
  {
    explicit StartData(const moveit::core::RobotModelConstPtr& robot_model)
      : start_state(robot_model)
    {
    }

    std::vector<std::string> joint_names;
    std::vector<std::string> variable_names;
    Eigen::VectorXd q_start;
    moveit::core::RobotState start_state;
  };

  struct GoalData
  {
    explicit GoalData(const moveit::core::RobotModelConstPtr& robot_model)
      : goal_state(robot_model)
    {
    }

    std::size_t goal_index{ 0 };
    moveit_msgs::msg::Constraints goal_raw;
    Eigen::VectorXd q_goal;  // same order as StartData.joint_names
    moveit::core::RobotState goal_state;
  };

private:
  StartData extractStart(const planning_scene::PlanningScene& scene,
                         const planning_interface::MotionPlanRequest& req,
                         const moveit::core::RobotModelConstPtr& robot_model,
                         const moveit::core::JointModelGroup& jmg) const;

  GoalData extractJointGoal(const planning_interface::MotionPlanRequest& req,
                            const moveit::core::JointModelGroup& jmg,
                            const StartData& start) const;

  std::shared_ptr<const trajopt::TrajArray>
  extractTrajectorySeed(const planning_interface::MotionPlanRequest& req,
                        const moveit::core::JointModelGroup& jmg) const;

  trajopt::TrajArray
  solveTrajOpt(const planning_interface::MotionPlanRequest& req,
               const std::shared_ptr<tesseract_environment::Environment>& env_snapshot,
               const StartData& start, const GoalData& goal,
               std::shared_ptr<const trajopt::TrajArray>& seed) const;

  static robot_trajectory::RobotTrajectoryPtr
  toMoveItTrajectory(const trajopt::TrajArray& trajopt_traj,
                     const moveit::core::RobotModelConstPtr& robot_model,
                     const std::string& group_name, const StartData& start);

  std::string
  resolveTesseractManipulatorGroup(const planning_interface::MotionPlanRequest& req) const;

  std::optional<SeedCacheEntry> getSeedHint(const std::string& key) const;
  void updateSeedHint(const std::string& key, SeedCacheEntry entry) const;

  std::optional<SolutionCacheEntry> getLastSolution(const std::string& key) const;
  void updateLastSolution(const std::string& key, SolutionCacheEntry entry) const;

private:
  TrajOptPlannerOptions::TrajOptOptions options_;

  mutable std::mutex config_mutex_;
  std::vector<rbt_planning::FeatureCost> feature_costs_;

  mutable std::mutex cache_mutex_;
  mutable std::unordered_map<std::string, SeedCacheEntry> seed_cache_;
  mutable std::unordered_map<std::string, SolutionCacheEntry> solution_cache_;
};

}  // namespace rlc_planner
