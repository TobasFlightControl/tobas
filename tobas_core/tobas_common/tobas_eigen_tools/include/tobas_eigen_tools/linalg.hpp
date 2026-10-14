// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>

#include "./core.hpp"

namespace tobas::eigen
{
/** Calculate the matrix rank. */
template <typename Derived>
inline Eigen::Index matrixRank(const Eigen::MatrixBase<Derived>& _a)
{
  return _a.fullPivLu().rank();
}

/** Check whether a square matrix is symmetric. */
template <typename Derived>
inline bool isSymmetric(const Eigen::MatrixBase<Derived>& _a)
{
  assert(isSquare(_a));
  return _a.isApprox(_a.transpose());
}

/** Check whether a square matrix is orthogonal. */
template <typename Derived>
inline bool isOrthogonal(const Eigen::MatrixBase<Derived>& _a)
{
  assert(isSquare(_a));
  return (_a * _a.transpose()).isApprox(Eigen::MatrixBase<Derived>::Identity());
}

/** Check whether a square matrix is special orthogonal. */
template <typename Derived>
inline bool isSpecialOrthogonal(const Eigen::MatrixBase<Derived>& _a)
{
  return isOrthogonal(_a) && math::isClose(_a.determinant(), 1.0);
}

/**
 * Check whether a matrix is positive definite.
 *
 * @see https://stackoverflow.com/questions/35227131/
 */
template <typename Derived>
inline bool isPositiveDefinite(const Eigen::MatrixBase<Derived>& _a)
{
  assert(isSquare(_a));
  return _a.llt().info() != Eigen::NumericalIssue;
}

/** Check whether a matrix is positive semidefinite. */
template <typename Derived>
bool isSemiPositiveDefinite(const Eigen::MatrixBase<Derived>& _a)
{
  assert(isSquare(_a));

  const auto ldlt = _a.ldlt();
  const auto D = ldlt.vectorD();
  const auto tol = std::numeric_limits<typename Derived::Scalar>::epsilon() * _a.cwiseAbs().maxCoeff() * 100;

  // `ldlt.isPositive()` may return false for tiny negative eigenvalues from numerical error.
  return D.minCoeff() >= -tol;
}

/** Check whether a matrix is symmetric positive definite. */
template <typename Derived>
inline bool isSymmetricPositiveDefinite(const Eigen::MatrixBase<Derived>& _a)
{
  return isSymmetric(_a) && isPositiveDefinite(_a);
}

/** Check whether a matrix is symmetric positive semidefinite. */
template <typename Derived>
inline bool isSymmetricSemiPositiveDefinite(const Eigen::MatrixBase<Derived>& _a)
{
  return isSymmetric(_a) && isSemiPositiveDefinite(_a);
}

/** Find the nearest positive definite matrix. */
template <typename Derived>
Derived nearestPositiveDefinite(const Eigen::MatrixBase<Derived>& _a, double _min_eigenvalue)
{
  assert(_min_eigenvalue >= 0);

  // Symmetrize.
  const Derived A_sym = (_a + _a.transpose()) / 2;

  // Eigendecomposition.
  const Eigen::SelfAdjointEigenSolver<Derived> es(A_sym);
  auto eigenvalues = es.eigenvalues();

  // Correct eigenvalues.
  for (Eigen::Index i = 0; i < eigenvalues.size(); ++i) {
    if (eigenvalues(i) < _min_eigenvalue) {
      eigenvalues(i) = _min_eigenvalue;
    }
  }

  // Reconstruct the corrected matrix.
  return es.eigenvectors() * eigenvalues.asDiagonal() * es.eigenvectors().transpose();
}

/**
 * Weighted squared-norm minimization.
 * minimize 0.5 ||Ax - b||^2_W1 + 0.5 ||x||^2_W2
 * <=> x = A^# b (A^# = (A^T W1 A + W2)^(-1) A^T W1) <- SR-inverse
 */
template <typename Scalar, Eigen::Index N, Eigen::Index M>
Eigen::Matrix<Scalar, M, 1> minimizeWeightedNorm(
  const Eigen::Matrix<Scalar, N, M>& _a,
  const Eigen::Matrix<Scalar, N, 1>& _b,
  const Eigen::Matrix<Scalar, N, 1>& _w1,
  const Eigen::Matrix<Scalar, M, 1>& _w2)
{
  assert((_w1.array() >= 0).all());
  assert((_w2.array() >= 0).all());

  // TODO: Use the Lagrange multiplier method for redundant problems.
  const Eigen::Matrix<Scalar, M, N> AT_W1 = _a.transpose() * _w1.asDiagonal();

  // Compute left matrix.
  Eigen::Matrix<Scalar, M, M> left = AT_W1 * _a;
  left.diagonal() += _w2;

  // Compute right vector.
  const auto right = AT_W1 * _b;

  // Solve linear equation.
  // QR decomposition does not give the least-squares solution for underdetermined problems.
  return left.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(right);
}
}  // namespace tobas::eigen
