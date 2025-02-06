#pragma once

#include <eigen3/Eigen/Core>

namespace ctrl
{
/**
 * @brief 離散時間代数リッカチ方程式の解を求める．
 */
Eigen::MatrixXd dare(
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& B,
  const Eigen::MatrixXd& Q,
  const Eigen::MatrixXd& R,
  const double& tol = 1e-3,
  size_t max_iter = 10000);
}  // namespace ctrl
