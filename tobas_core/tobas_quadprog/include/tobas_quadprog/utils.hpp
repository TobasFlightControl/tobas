#pragma once

#include <eigen3/Eigen/Core>

namespace quadprog
{
/**
 * @brief 変数ベクトルxの範囲(lb <= x <= ub)から等価な行列不等式(A @ x <= b)を作る．
 *
 * @param lb 下限
 * @param ub 上限
 * @param inf これ以上の値を行列不等式から省く
 *
 * @return LinearEquation A @ x <= b の(A, b)
 */
void matIneqFromRange(
  const Eigen::VectorXd& lb,
  const Eigen::VectorXd& ub,
  Eigen::MatrixXd& A,
  Eigen::VectorXd& b,
  const double inf = 1E+12);
}  // namespace quadprog
