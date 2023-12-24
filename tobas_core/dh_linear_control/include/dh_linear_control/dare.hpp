#pragma once

#include <Eigen/Core>

namespace ctrl
{
enum DareMethod
{
  Normal,
  Joseph,
};

/**
 * @brief 離散時間代数リッカチ方程式の解を求める．
 */
Eigen::MatrixXd dare(
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& B,
  const Eigen::MatrixXd& Q,
  const Eigen::MatrixXd& R,
  DareMethod method = DareMethod::Normal,
  const double& tol = 1e-3,
  size_t max_iter = 10000);
}  // namespace ctrl
