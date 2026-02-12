#pragma once

/**
 * @file trajopt_solver.hpp
 * @brief TrajOpt-based trajectory optimization wrapper for joint-space planning.
 */

#include <tesseract_environment/environment.h>

#include <Eigen/Core>
#include <functional>
#include <memory>
#include <string>
#include <trajopt/problem_description.hpp>
#include <trajopt/utils.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/sco_common.hpp>
#include <vector>
namespace rbt_planning
{

/**
 * @brief User-defined cost evaluated on the full trajectory.
 *
 * @details
 * The callback `phi` is evaluated on the current trajectory iterate and scaled by
 * `weight`. The trajectory is represented as a matrix with:
 * - rows = timesteps (`n_steps`)
 * - cols = joint DOF (`ndof`)
 *
 * Joint ordering must be consistent with the active TrajOpt problem (i.e., the configured
 * manipulator group).
 *
 * @note `grad` is currently not used by `TrajOptSolver`; it is provided for future
 * analytic-gradient support.
 */
struct FeatureCost
{
  std::string name;  ///< Cost term name (used for TrajOpt/SCO bookkeeping and debugging).
  double weight;     ///< Scalar multiplier applied to `phi(traj)`.
  std::function<double(const trajopt::TrajArray&)>
      phi;  ///< Cost function evaluated on `traj`.
  std::function<Eigen::VectorXd(const trajopt::TrajArray&)>
      grad;  ///< Optional analytic gradient (unused).
};

/**
 * @brief Stateful TrajOpt solver for joint-space trajectory optimization.
 *
 * @details
 * This class builds a TrajOpt problem using the current configuration (environment,
 * manipulator group, number of steps, start/goal states, optional seed) and solves it
 * using a trust-region SQP optimizer.
 *
 * Unless otherwise stated, joint vectors (`q_start`, `q_goal`) are joint positions [rad]
 * in the ordering expected by the configured manipulator group, with `size = ndof`.
 *
 * @par Usage
 * @code{.cpp}
 * auto env = std::make_shared<tesseract_environment::Environment>();
 * // Configure env (e.g., load URDF/SRDF, set initial state, ...).
 *
 * rbt_planning::TrajOptSolver solver(env);
 * solver.setManipulatorGroup("manipulator");
 * solver.setNumSteps(20);
 * solver.setStartState(joint_names, q_start);
 * solver.setGoalState(q_goal);
 *
 * trajopt::TrajArray traj = solver.solve();
 * @endcode
 *
 * @par Thread safety
 * Not thread-safe.
 */
class TrajOptSolver
{
public:
  /**
   * @brief Constructs a solver with an optional environment.
   * @param[in] env Tesseract environment used to build TrajOpt problems; may be nullptr.
   */
  explicit TrajOptSolver(std::shared_ptr<tesseract_environment::Environment> env = nullptr);

  /**
   * @brief Returns the optimizer status from the last call to solve().
   * @return Last optimizer status.
   * @note Returns `sco::OptStatus::INVALID` until solve() has been called.
   */
  sco::OptStatus lastStatus() const
  {
    return last_status_;
  }

  /**
   * @brief Returns the last constructed TrajOpt problem.
   * @return Shared pointer to the last problem; may be nullptr if solve() has not been called.
   */
  std::shared_ptr<trajopt::TrajOptProb> lastProblem() const
  {
    return prob_;
  }

  /**
   * @brief Sets the Tesseract environment used to construct problems.
   * @param[in] env Environment; may be nullptr.
   */
  void setEnvironment(std::shared_ptr<tesseract_environment::Environment> env);

  /**
   * @brief Sets the manipulator group used by TrajOpt.
   * @param[in] group Manipulator group name (must exist in the environment).
   */
  void setManipulatorGroup(std::string group);

  /**
   * @brief Sets the number of timesteps in the optimized trajectory.
   * @param[in] n_steps Number of timesteps (rows in the trajectory).
   * @throws std::invalid_argument If `n_steps < 2`.
   */
  void setNumSteps(int n_steps);

  /**
   * @brief Sets the start joint state and updates the environment state.
   * @param[in] joint_names Joint names corresponding to `q_start`, size = `q_start.size()`.
   * @param[in] q_start Start joint positions [rad], size = `joint_names.size()`.
   * @throws std::runtime_error If the environment is not set.
   * @throws std::invalid_argument If `joint_names.size() != q_start.size()`.
   * @note `joint_names` ordering must match `q_start` and should be compatible with the
   * active manipulator group.
   */
  void setStartState(const std::vector<std::string>& joint_names,
                     const Eigen::VectorXd& q_start);

  /**
   * @brief Sets the goal joint state.
   * @param[in] q_goal Goal joint positions [rad], size = `q_start.size()`.
   * @throws std::runtime_error If setStartState() has not been called.
   * @throws std::invalid_argument If `q_goal.size() != q_start.size()`.
   */
  void setGoalState(const Eigen::VectorXd& q_goal);

  /**
   * @brief Sets an explicit initial trajectory seed for the optimizer.
   * @param[in] traj Seed trajectory of joint positions [rad], shape = (`n_steps`, `ndof`).
   * @throws std::invalid_argument If `traj->rows() != n_steps` or `traj->cols() != ndof`.
   * @pre setStartState() has been called so that `ndof` is defined.
   * @note If not set (or empty), solve() uses a linear interpolation between start and goal.
   */
  void setTrajSeed(std::shared_ptr<const trajopt::TrajArray> traj);

  /**
   * @brief Adds a user-defined cost term to the TrajOpt problem.
   * @param[in] cost Cost definition (stored internally by value).
   * @throws std::invalid_argument If `cost.phi` is empty.
   */
  void addFeatureCost(FeatureCost cost);

  /**
   * @brief Constructs and solves the TrajOpt problem.
   * @return Optimized joint trajectory [rad], shape = (`n_steps`, `ndof`).
   * @throws std::runtime_error If the environment is not set or start/goal states are missing.
   * @throws std::invalid_argument If the configured seed trajectory has invalid shape.
   * @note Updates lastStatus() and lastProblem().
   */
  trajopt::TrajArray solve();

private:
  void constructProblem();
  void validateTrajectory(std::shared_ptr<const trajopt::TrajArray>& traj) const;
  trajopt::TrajArray makeLinearSeed(const Eigen::VectorXd& q0, const Eigen::VectorXd& q1,
                                    int n_steps) const;
  static sco::VarVector getVars(trajopt::TrajOptProb& prob);
  void attachFeatureCosts(trajopt::TrajOptProb& prob) const;

private:
  std::shared_ptr<tesseract_environment::Environment> env_;

  std::string manipulator_group_{ "manipulator" };
  int n_steps_{ 20 };

  std::vector<std::string> joint_names_;
  Eigen::VectorXd q_start_;
  Eigen::VectorXd q_goal_;
  std::shared_ptr<const trajopt::TrajArray> traj_seed_;

  std::shared_ptr<trajopt::TrajOptProb> prob_;
  sco::OptStatus last_status_{ sco::OptStatus::INVALID };
  std::vector<FeatureCost> feature_costs_;
};

}  // namespace rbt_planning
