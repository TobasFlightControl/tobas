#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <Eigen/Eigen>

#include "./core.hpp"

namespace eigen_tools
{
/* 行列のランクを計算する． */
template <typename Derived>
inline Eigen::Index matrixRank(const Eigen::MatrixBase<Derived>& A)
{
  return A.fullPivLu().rank();
}

/**
 * @brief 行列が正定値行列かどうかを判定する．
 * cf. https://stackoverflow.com/questions/35227131/
 */
template <typename Derived>
inline bool isPositiveDefinite(const Eigen::MatrixBase<Derived>& A)
{
  assert(isSquare(A));
  return A.llt().info() != Eigen::NumericalIssue;
}

/**
 * @brief 行列が半正定値行列かどうかを判定する．
 * cf. file:///home/dohi/Downloads/081791add.pdf
 */
template <typename Derived>
bool isSemiPositiveDefinite(const Eigen::MatrixBase<Derived>& A)
{
  assert(isSymmetric(A));  // 対称行列でないと判定できない

  const Eigen::SelfAdjointEigenSolver<Derived> es(A);
  const auto min_eigenvalue = es.eigenvalues()(0);
  return min_eigenvalue >= 0;
}

/* 行列が正定値対象行列かどうかを判定する． */
template <typename Derived>
inline bool isSymmetricPositiveDefinite(const Eigen::MatrixBase<Derived>& A)
{
  return isSymmetric(A) && isPositiveDefinite(A);
}

/* 行列が半正定値対象行列かどうかを判定する． */
template <typename Derived>
inline bool isSymmetricSemiPositiveDefinite(const Eigen::MatrixBase<Derived>& A)
{
  return isSymmetric(A) && isSemiPositiveDefinite(A);
}

/**
 * @brief 重み付き二乗ノルム最小化．
 * minimize 0.5 ||Ax - b||^2_W1 + 0.5 ||x||^2_W2
 * <=> x = A^# b (A^# = (A^T W1 A + W2)^(-1) A^T W1) <- SR-inverse
 */
template <typename Scalar, Eigen::Index N, Eigen::Index M>
Eigen::Matrix<Scalar, M, 1> minimizeWeightedNorm(
  const Eigen::Matrix<Scalar, N, M>& A,
  const Eigen::Matrix<Scalar, N, 1>& b,
  const Eigen::Matrix<Scalar, N, 1>& W1,
  const Eigen::Matrix<Scalar, M, 1>& W2)
{
  assert((W1.array() >= 0).all());
  assert((W2.array() >= 0).all());

  // TODO: 冗長問題の場合はラグランジュの未定乗数法
  const Eigen::Matrix<Scalar, M, N> AT_W1 = A.transpose() * W1.asDiagonal();

  // Compute left matrix
  Eigen::Matrix<Scalar, M, M> left = AT_W1 * A;
  left.diagonal() += W2;

  // Compute right vector
  const auto right = AT_W1 * b;

  // Solve linear equation
  // NOTE: QR分解は決定不全問題の最小二乗解を与えない
  return left.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(right);
}
}  // namespace eigen_tools
