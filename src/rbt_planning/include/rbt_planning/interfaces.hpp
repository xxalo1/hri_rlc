// include/rbt_planning/interfaces.hpp
#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

#include "rbt_planning/types.hpp"

namespace rbt_planning {

class CollisionWorldInterface {
 public:
  virtual ~CollisionWorldInterface() = default;

  virtual const std::vector<std::string>& jointNames() const = 0;

  virtual bool inCollision(const Eigen::VectorXd& q) const = 0;

  virtual double minDistance(const Eigen::VectorXd& q) const = 0;
};

class PlannerInterface {
 public:
  virtual ~PlannerInterface() = default;

  virtual std::string name() const = 0;

  virtual PlanResult plan(const SceneSnapshot& scene, const JointGoal& goal,
                          const ProblemSpec& spec) = 0;
};

}  // namespace rbt_planning
