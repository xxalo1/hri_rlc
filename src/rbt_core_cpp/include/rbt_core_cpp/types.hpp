#pragma once

#include <Eigen/Core>

namespace rbt_core_cpp {

using Vec = Eigen::VectorXd;
using Mat = Eigen::MatrixXd;
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat4 = Eigen::Matrix4d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat6N = Eigen::Matrix<double, 6, Eigen::Dynamic>;

} // namespace rbt_core_cpp
