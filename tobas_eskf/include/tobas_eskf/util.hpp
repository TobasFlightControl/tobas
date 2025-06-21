#pragma once

#include <eigen3/Eigen/Core>

namespace eskf
{
/* 地磁気の分散から方位角の分散を推定する (memo: 2-75) */
double headVarianceFromMag(const Eigen::Vector3d& mag, const Eigen::Matrix3d& cov);
}  // namespace eskf
