#pragma once

#include <tesseract_environment/environment.h>

#include <Eigen/Core>
#include <memory>
#include <string>
#include <trajopt/problem_description.hpp>
#include <trajopt/utils.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/sco_common.hpp>
#include <utility>
#include <vector>

namespace rbt_planning {

class TrajoptPlanner {
 public:
  explicit TrajoptPlanner(
      std::shared_ptr<tesseract_environment::Environment> env = nullptr);

  sco::OptStatus last_status() const { return last_status_; }

  boost::shared_ptr<trajopt::TrajOptProb> last_problem() const {
    return last_prob_;
  }
  void set_environment(std::shared_ptr<tesseract_environment::Environment> env);

  void set_manipulator_group(std::string group);
  void set_num_steps(int n_steps);

  void set_start_state(const std::vector<std::string>& joint_names,
                       const Eigen::VectorXd& q_start);
  void set_goal_state(const Eigen::VectorXd& q_goal);
  void set_traj_seed(trajopt::TrajArray&& seed);

  trajopt::TrajArray solve();

  boost::shared_ptr<trajopt::TrajOptProb> last_problem() const;
  sco::OptStatus last_status() const;

 private:
  void construct_problem() const;
  void validate_traj(const trajopt::TrajArray& seed) const;
  static trajopt::TrajArray make_linear_seed(const Eigen::VectorXd& q0,
                                             const Eigen::VectorXd& q1,
                                             int n_steps);

 private:
  std::shared_ptr<tesseract_environment::Environment> env_;

  std::string manipulator_group_{"manipulator"};
  int n_steps_{20};

  std::vector<std::string> joint_names_;
  Eigen::VectorXd q_start_;
  Eigen::VectorXd q_goal_;
  trajopt::TrajArray traj_seed_;

  boost::shared_ptr<trajopt::TrajOptProb> prob_;
  sco::OptStatus last_status_{sco::OptStatus::INVALID};
};

}  // namespace rbt_planning
