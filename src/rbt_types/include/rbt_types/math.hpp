#pragma once

/**
 * @file math.hpp
 * @brief Shared Eigen aliases used across `rbt_*` packages.
 */

#include <Eigen/Core>

namespace rbt_types
{

/// @brief Dynamic-size column vector (`Eigen::VectorXd`).
using Vec = Eigen::VectorXd;

/// @brief Dynamic-size matrix (`Eigen::MatrixXd`).
using Mat = Eigen::MatrixXd;

/// @brief Joint-space state vector [rad], size = dof.
using JointVec = Vec;

/// @brief Reward/IRL parameter vector, size = num_features.
using WeightVec = Vec;

/// @brief Feature-count/value vector, size = num_features.
using FeatureVec = Vec;

/// @brief Row-major trajectory matrix, shape = (num_waypoints, dof).
using TrajectoryMat =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

}  // namespace rbt_types
