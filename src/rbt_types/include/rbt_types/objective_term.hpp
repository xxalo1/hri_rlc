#pragma once

#include <vector>

#include "rbt_types/feature.hpp"

namespace rbt_types
{

struct ObjectiveTerm
{
  FeaturePtr feature;  // shared_ptr<const Feature>
  double weight{ 1.0 };
};

using RewardTerm = ObjectiveTerm;
using CostTerm = ObjectiveTerm;

using ObjectiveTerms = std::vector<ObjectiveTerm>;
using RewardTerms = ObjectiveTerms;
using CostTerms = ObjectiveTerms;

inline void negateWeightsInPlace(ObjectiveTerms& terms)
{
  for (auto& t : terms)
  {
    t.weight = -t.weight;
  }
}

[[nodiscard]] inline CostTerms rewardToCost(const RewardTerms& reward_terms)
{
  CostTerms cost_terms = reward_terms;
  negateWeightsInPlace(cost_terms);
  return cost_terms;
}

[[nodiscard]] inline RewardTerms costToReward(const CostTerms& cost_terms)
{
  RewardTerms reward_terms = cost_terms;
  negateWeightsInPlace(reward_terms);
  return reward_terms;
}

inline double evaluateObjective(const ObjectiveTerms& terms, const Trajectory& traj)
{
  double total = 0.0;
  for (const auto& t : terms)
  {
    total += t.weight * t.feature->evaluate(traj);
  }
  return total;
}

}  // namespace rbt_types