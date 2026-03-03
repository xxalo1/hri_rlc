#include "rbt_irl/max_ent.hpp"

#include <stdexcept>

#include "rbt_irl/utils.hpp"

namespace rbt_irl
{

MaxEntIRL::MaxEntIRL(const FeatureList& features, const SamplerFn& sampler)
  : MaxEntIRL(features, sampler, Options{})
{
}

MaxEntIRL::MaxEntIRL(const FeatureList& features, const SamplerFn& sampler,
                     const Options& opt)
  : sampler_(sampler), opt_(opt), features_(features)
{
}

MaxEntIRL::FeatureVec MaxEntIRL::computeFeatureCounts(const Trajectory& traj) const
{
  const Eigen::Index k = featureDim();
  FeatureVec phi = FeatureVec::Zero(k);

  Eigen::Index i = 0;
  for (const auto& f : features_)
  {
    const auto v = f->evaluate(traj);
    phi[i] = v;
    i += 1;
  }
  return phi;
}

MaxEntIRL::FeatureVec MaxEntIRL::computeFeatureMean(const TrajectorySet& trajs) const
{
  const Eigen::Index k = featureDim();
  FeatureVec sum = FeatureVec::Zero(k);

  for (const auto& traj : trajs)
  {
    sum += computeFeatureCounts(traj);
  }

  return sum / static_cast<double>(trajs.size());
}

MaxEntIRL::TrajectorySet
MaxEntIRL::sampleUnderTheta(const TrajectoryEndpointsSet& demo_endpoints,
                            const WeightVec& theta) const
{
  RewardTerms rewards;
  rewards.reserve(featureDim());

  for (std::size_t i = 0; i < features_.size(); ++i)
  {
    rewards.push_back({ features_[i], theta[i] });
  }

  TrajectorySet sampled_trajs;
  sampled_trajs.reserve(demo_endpoints.size());

  for (const auto& sgs : demo_endpoints)
  {
    auto traj = sampler_(sgs, rewards);
    sampled_trajs.push_back(std::move(traj));
  }
  return sampled_trajs;
}

MaxEntIRL::WeightVec MaxEntIRL::fit(const TrajectorySet& demo_trajs,
                                    const WeightVec& theta0)
{
  validate(demo_trajs);

  initializeTheta(theta0);

  const TrajectoryEndpointsSet demo_endpoints =
      utils::extractTrajectoryEndpoints(demo_trajs);

  const FeatureVec emp_feat_mean = computeFeatureMean(demo_trajs);

  for (int it = 0; it < opt_.max_iters; ++it)
  {
    const WeightVec theta_prev = theta_;
    
    const TrajectorySet sampled_trajs = sampleUnderTheta(demo_endpoints, theta_);
    const FeatureVec exp_feat_mean = computeFeatureMean(sampled_trajs);
    const WeightVec grad = estimateGradient(emp_feat_mean, exp_feat_mean);

    theta_ -= opt_.learning_rate * grad;

    if (hasConverged(theta_prev, theta_))
    {
      break;
    }
  }

  return theta_;
}

void MaxEntIRL::initializeTheta(const WeightVec& theta0)
{
  const Eigen::Index k = static_cast<Eigen::Index>(featureDim());
  if (theta0.size() == 0)
  {
    theta_ = WeightVec::Ones(k);
  }
  else if (theta0.size() != k)
  {
    throw std::invalid_argument("theta0 wrong size");
  }
  else
  {
    theta_ = theta0;
  }
}

MaxEntIRL::WeightVec MaxEntIRL::estimateGradient(const FeatureVec& emp_mean,
                                                 const FeatureVec& exp_mean) const
{
  return emp_mean - exp_mean;
}

bool MaxEntIRL::hasConverged(const WeightVec& theta_prev, const WeightVec& theta) const
{
  return (theta - theta_prev).norm() < opt_.convergence_tol;
}

void MaxEntIRL::validate(const TrajectorySet& demos) const
{
  if (features_.empty())
  {
    throw std::invalid_argument("MaxEntIRL: features list is empty.");
  }
  if (demos.empty())
  {
    throw std::invalid_argument("MaxEntIRL: demonstrations set is empty.");
  }
  if (!sampler_)
  {
    throw std::invalid_argument("MaxEntIRL: sampler is not set.");
  }

  const Eigen::Index k = static_cast<Eigen::Index>(featureDim());
  if (theta_.size() != k)
  {
    throw std::invalid_argument("MaxEntIRL: theta has wrong dimension.");
  }
}

}  // namespace rbt_irl
