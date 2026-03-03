#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include "rbt_types/trajectory.hpp"

namespace rbt_types
{

class Feature
{
public:
  virtual ~Feature() = default;

  virtual std::string name() const = 0;

  virtual double evaluate(const Trajectory& traj) const = 0;
};

using FeaturePtr = std::shared_ptr<const Feature>;
using Features = std::vector<FeaturePtr>;

}  // namespace rbt_types
