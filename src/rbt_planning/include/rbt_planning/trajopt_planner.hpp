#pragma once

#include <tesseract_common/joint_state.h>
#include <tesseract_environment/environment.h>

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

namespace rbt_planning {

struct TrajoptRequest {
  std::vector<std::string> joint_names;
  Eigen::VectorXd q_start;
  Eigen::VectorXd q_goal;

  std::string manipulator = "manipulator";
  std::string profile = "freespace_profile";
};

class TrajoptPlanner {
 public:
  explicit TrajoptPlanner(
      std::shared_ptr<const tesseract_environment::Environment> env);

  tesseract_common::JointTrajectory plan(const TrajoptRequest& req) const;

 private:
  std::shared_ptr<const tesseract_environment::Environment> env_;
};

}  // namespace rbt_planning
