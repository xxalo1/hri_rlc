// include/rbt_planning/types.hpp
#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

namespace rbt_planning {

using JointNameList = std::vector<std::string>;

using JointVector = Eigen::VectorXd;

using JointTrajectory = Eigen::MatrixXd;

class CollisionWorldInterface;

struct SceneSnapshot {
  JointNameList joint_names;
  JointVector q_start;
  std::shared_ptr<const CollisionWorldInterface> collision_world;
  std::string frame_id;
};

struct JointGoal {
  JointVector q_goal;
};

struct ProblemSpec {
  int num_waypoints = 20;

  bool enable_collision = true;
  bool enable_joint_limits = true;

  double w_smoothness = 1.0;

  double w_collision = 0.0;
  double collision_margin = 0.02;
};

enum class PlanStatus {
  kSuccess = 0,
  kInvalidInput,
  kInfeasible,
  kSolverFailure
};

struct PlanDiagnostics {
  std::string solver_name;
  int iterations = 0;
  double solve_time_s = 0.0;
};

struct PlanResult {
  PlanStatus status = PlanStatus::kSolverFailure;
  std::string message;
  JointTrajectory q_traj;
  PlanDiagnostics diag;
};

}  // namespace rbt_planning
