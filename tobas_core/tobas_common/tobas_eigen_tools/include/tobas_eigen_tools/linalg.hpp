// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>

#include "./core.hpp"

namespace tobas
{
namespace eigen
{
/* Calculate the matrix rank. */
template <typename Derived>
inline Eigen::Index matrixRank(const Eigen::MatrixBase<Derived>& A)
{
  return A.fullPivLu().rank();
}

/* Check whether a square matrix is symmetric. */
template <typename Derived>
inline bool isSymmetric(const Eigen::MatrixBase<Derived>& A)
{
  assert(isSquare(A));
  return A.isApprox(A.transpose());
}

/* Check whether a square matrix is orthogonal. */
template <typename Derived>
inline bool isOrthogonal(const Eigen::MatrixBase<Derived>& A)
{
  assert(isSquare(A));
  return (A * A.transpose()).isApprox(Eigen::MatrixBase<Derived>::Identity());
}

/* Check whether a square matrix is special orthogonal. */
template <typename Derived>
inline bool isSpecialOrthogonal(const Eigen::MatrixBase<Derived>& A)
{
  return isOrthogonal(A) && math::isClose(A.determinant(), 1.0);
}

/**
 * @brief Check whether a matrix is positive definite.
 * cf. https://stackoverflow.com/questions/35227131/
 */
template <typename Derived>
inline bool isPositiveDefinite(const Eigen::MatrixBase<Derived>& A)
{
  assert(isSquare(A));
  return A.llt().info() != Eigen::NumericalIssue;
}

/* Check whether a matrix is positive semidefinite. */
template <typename Derived>
bool isSemiPositiveDefinite(const Eigen::MatrixBase<Derived>& A)
{
  assert(isSquare(A));

  const auto ldlt = A.ldlt();
  const auto D = ldlt.vectorD();
  const auto tol = std::numeric_limits<typename Derived::Scalar>::epsilon() * A.cwiseAbs().maxCoeff() * 100;

  // `ldlt.isPositive()` may return false for tiny negative eigenvalues from numerical error.
  return D.minCoeff() >= -tol;
}

/* Check whether a matrix is symmetric positive definite. */
template <typename Derived>
inline bool isSymmetricPositiveDefinite(const Eigen::MatrixBase<Derived>& A)
{
  return isSymmetric(A) && isPositiveDefinite(A);
}

/* Check whether a matrix is symmetric positive semidefinite. */
template <typename Derived>
inline bool isSymmetricSemiPositiveDefinite(const Eigen::MatrixBase<Derived>& A)
{
  return isSymmetric(A) && isSemiPositiveDefinite(A);
}

/* Find the nearest positive definite matrix. */
template <typename Derived>
Derived nearestPositiveDefinite(const Eigen::MatrixBase<Derived>& A, double min_eigenvalue)
{
  assert(min_eigenvalue >= 0);

  // Symmetrize.
  const Derived A_sym = (A + A.transpose()) / 2;

  // Eigendecomposition.
  const Eigen::SelfAdjointEigenSolver<Derived> es(A_sym);
  auto eigenvalues = es.eigenvalues();

  // Correct eigenvalues.
  for (Eigen::Index i = 0; i < eigenvalues.size(); ++i) {
    if (eigenvalues(i) < min_eigenvalue) {
      eigenvalues(i) = min_eigenvalue;
    }
  }

  // Reconstruct the corrected matrix.
  return es.eigenvectors() * eigenvalues.asDiagonal() * es.eigenvectors().transpose();
}

/**
 * @brief Weighted squared-norm minimization.
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

  // TODO: Use the Lagrange multiplier method for redundant problems.
  const Eigen::Matrix<Scalar, M, N> AT_W1 = A.transpose() * W1.asDiagonal();

  // Compute left matrix.
  Eigen::Matrix<Scalar, M, M> left = AT_W1 * A;
  left.diagonal() += W2;

  // Compute right vector.
  const auto right = AT_W1 * b;

  // Solve linear equation.
  // QR decomposition does not give the least-squares solution for underdetermined problems.
  return left.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(right);
}
}  // namespace eigen
}  // namespace tobas
