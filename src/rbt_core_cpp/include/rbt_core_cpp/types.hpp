#pragma once

/**
 * @file types.hpp
 * @brief Eigen type aliases used by `rbt_core_cpp`.
 */

#include <Eigen/Core>

namespace rbt_core_cpp {

/// @brief Dynamic-size column vector (`Eigen::VectorXd`).
using Vec = Eigen::VectorXd;

/// @brief Dynamic-size matrix (`Eigen::MatrixXd`).
using Mat = Eigen::MatrixXd;

/// @brief 3D vector (`Eigen::Vector3d`).
using Vec3 = Eigen::Vector3d;

/// @brief 6D vector, often used for spatial quantities [vx vy vz wx wy wz].
using Vec6 = Eigen::Matrix<double, 6, 1>;

/// @brief 4x4 homogeneous transform matrix.
using Mat4 = Eigen::Matrix4d;

/// @brief 6x6 matrix, often used for task-space gains.
using Mat6 = Eigen::Matrix<double, 6, 6>;

/// @brief 6xN matrix for Jacobians, shape = (6, cols).
using Mat6N = Eigen::Matrix<double, 6, Eigen::Dynamic>;

}
