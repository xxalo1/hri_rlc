#pragma once

#include <cstddef>
#include <functional>

#include "rbt_types/feature.hpp"
#include "rbt_types/math.hpp"
#include "rbt_types/trajectory.hpp"
#include "rbt_types/objective_term.hpp"
#include "rbt_irl/utils.hpp"

namespace rbt_irl
{

class MaxEntIRL
{
public:
  using Trajectory = rbt_types::Trajectory;
  using TrajectorySet = rbt_types::TrajectorySet;

  using FeaturePtr = rbt_types::FeaturePtr;
  using FeatureList = rbt_types::Features;

  using WeightVec = rbt_types::WeightVec;
  using FeatureVec = rbt_types::FeatureVec;

  using RewardTerms = rbt_types::RewardTerms;

  using TrajectoryEndpoints = utils::TrajectoryEndpoints;
  using TrajectoryEndpointsSet = utils::TrajectoryEndpointsSet;

  using SamplerFn = std::function<Trajectory(const TrajectoryEndpoints& start_goal,
                                             const RewardTerms& reward_terms)>;

  /**
   * @brief Configuration options for the MaxEnt IRL optimizer.
   *
   * @details
   * - `learning_rate`: Gradient update step size (unitless).
   * - `max_iters`: Maximum number of optimization iterations.
   * - `convergence_tol`: Stop when `||theta_k - theta_{k-1}|| < convergence_tol`.
   */
  struct Options
  {
    double learning_rate{ 1.0 };
    int max_iters{ 50 };
    double convergence_tol{ 1e-4 };
  };

  /**
   * @brief Constructs a solver with default options.
   * @param[in] features List of feature functions.
   * @param[in] sampler Trajectory sampler used to generate trajectories under the current weights.
   */
  MaxEntIRL(const FeatureList& features, const SamplerFn& sampler);

  /**
   * @brief Constructs a solver with explicitly provided options.
   * @param[in] features List of feature functions used to compute the feature vector.
   * @param[in] sampler Trajectory sampler used to generate trajectories under the current weights.
   * @param[in] opt Optimization options.
   */
  MaxEntIRL(const FeatureList& features, const SamplerFn& sampler, const Options& opt);

  /**
   * @brief Learns weights from demonstration trajectories.
   * @param[in] demos Demonstration trajectories, length >= 1.
   * @param[in] theta0 Optional initial weights; if empty, initializes to `Ones(features.size())`.
   * @return Learned weights `theta`, size = features.size().
   */
  WeightVec fit(const TrajectorySet& demo_trajs, const WeightVec& theta0 = WeightVec{});

  const WeightVec& theta() const
  {
    return theta_;
  }
  void setTheta(const WeightVec& t)
  {
    theta_ = t;
  }

  void setFeatures(const FeatureList& feats)
  {
    features_ = feats;
  }
  void setOptions(const Options& opt)
  {
    opt_ = opt;
  }

private:
  FeatureVec computeFeatureCounts(const Trajectory& traj) const;

  FeatureVec computeFeatureMean(const TrajectorySet& trajs) const;

  TrajectorySet sampleUnderTheta(const TrajectoryEndpointsSet& demo_endpoints,
                                 const WeightVec& theta) const;

  WeightVec estimateGradient(const FeatureVec& demo_mean,
                             const FeatureVec& expected_mean) const;

  bool hasConverged(const WeightVec& theta_prev, const WeightVec& theta) const;

  void validateConfiguration() const;
  static void validateDemos(const TrajectorySet& demos);

  void initializeTheta(const WeightVec& theta0);

  std::size_t featureDim() const
  {
    return features_.size();
  }

  SamplerFn sampler_;  // injected
  Options opt_;
  FeatureList features_;
  WeightVec theta_;
};

}  // namespace rbt_irl
