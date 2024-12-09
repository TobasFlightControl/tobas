#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Eigen>

#include "./core.hpp"

namespace eigen
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
  assert(isSquare(A));
  const auto ldlt = A.ldlt();
  const auto D = ldlt.vectorD();
  const auto tol = std::numeric_limits<typename Derived::Scalar>::epsilon() * A.cwiseAbs().maxCoeff() * 100;
  return D.minCoeff() >= -tol;  // XXX: ldlt.isPositive()は数値誤差で極小の負の固有値が含まれる際にfalseを返してしまう
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

/* 最近接正定行列を求める． */
template <typename Derived>
Derived nearestPositiveDefinite(const Eigen::MatrixBase<Derived>& A, double min_eigenvalue)
{
  assert(min_eigenvalue >= 0);

  // 対称化
  const Derived A_sym = (A + A.transpose()) / 2;

  // 固有値分解
  Eigen::SelfAdjointEigenSolver<Derived> es(A_sym);
  auto eigenvalues = es.eigenvalues();

  // 固有値を修正
  for (Eigen::Index i = 0; i < eigenvalues.size(); ++i)
    if (eigenvalues(i) < min_eigenvalue)
      eigenvalues(i) = min_eigenvalue;

  // 修正後の行列を再構築
  return es.eigenvectors() * eigenvalues.asDiagonal() * es.eigenvectors().transpose();
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
}  // namespace eigen
