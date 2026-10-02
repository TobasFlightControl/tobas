// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/care.hpp"

#include <eigen3/Eigen/LU>

#include <tobas_eigen_tools/linalg.hpp>

#include "tobas_control/util.hpp"

namespace tobas
{
namespace ctrl
{
std::expected<Eigen::MatrixXd, std::string> care_ArimotoPotter(
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& B,
  const Eigen::MatrixXd& Q,
  const Eigen::MatrixXd& R) noexcept
{
  const auto n = A.rows();
  [[maybe_unused]] const auto l = B.cols();

  assert(A.rows() == n && A.cols() == n);
  assert(eigen::isFinite(A));

  assert(B.rows() == n && B.cols() == l);
  assert(eigen::isFinite(B));

  assert(Q.rows() == n && Q.cols() == n);
  assert(eigen::isFinite(Q));
  assert(eigen::isSemiPositiveDefinite(Q));

  assert(R.rows() == l && R.cols() == l);
  assert(eigen::isFinite(R));
  assert(eigen::isPositiveDefinite(R));

  // TODO: Check stabilizability.

  // Make Hamilton matrix.
  Eigen::MatrixXd H(n * 2, n * 2);
  H.topLeftCorner(n, n) = A;
  H.topRightCorner(n, n) = -B * R.inverse() * B.transpose();
  H.bottomLeftCorner(n, n) = -Q;
  H.bottomRightCorner(n, n) = -A.transpose();

  // Get eigenvalues and eigenvectors.
  const Eigen::EigenSolver<Eigen::MatrixXd> es(H);
  if (es.info() != Eigen::Success) {
    return std::unexpected("Failed to get eigenvalues.");
  }
  const auto eigvals = es.eigenvalues().real().eval();
  const auto eigvecs = es.eigenvectors().eval();  // Eigenvectors require the imaginary parts too.

  // The number of eigenvalues with negative real parts should match the order of the system.
  // Use a small margin because comparing with 0 can catch uninitialized values.
  const auto num_stable_eigvals = (eigvals.array() < -std::numeric_limits<double>::epsilon()).count();
  if (num_stable_eigvals != n) {
    return std::unexpected("The number of stable eigenvalues does not match the order of the system.");
  }

  // Extract eigenvectors corresponding to stable eigenvalues.
  Eigen::MatrixXcd eigvecs_stable(n * 2, n);
  Eigen::Index j = 0;  // The index of stable eigenvalue. This value must become identical to n.
  for (Eigen::Index i = 0; i < n * 2; ++i) {
    if (eigvals(i) < 0.0) {
      eigvecs_stable.col(j) = eigvecs.col(i);
      ++j;
    }
  }
  assert(j == n);

  // Compute P with stable eigen vector matrix.
  const auto Y = eigvecs_stable.block(0, 0, n, n);
  const auto Z = eigvecs_stable.block(n, 0, n, n);
  return (Z * Y.inverse()).real().eval();
}

Eigen::MatrixXd
care_Schur(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
{
  const auto n = A.rows();
  [[maybe_unused]] const auto l = B.cols();

  assert(A.rows() == n && A.cols() == n);
  assert(B.rows() == n && B.cols() == l);
  assert(Q.rows() == n && Q.cols() == n);
  assert(R.rows() == l && R.cols() == l);

  assert(eigen::isFinite(A));
  assert(eigen::isFinite(B));
  assert(eigen::isFinite(Q));
  assert(eigen::isFinite(R));

  assert(isControllable(A, B));
  assert(eigen::isSemiPositiveDefinite(Q));
  assert(eigen::isPositiveDefinite(R));

  Eigen::MatrixXd H(n * 2, n * 2);
  H.topLeftCorner(n, n) = A;
  H.topRightCorner(n, n) = -B * R.inverse() * B.transpose();
  H.bottomLeftCorner(n, n) = -Q;
  H.bottomRightCorner(n, n) = -A.transpose();

  const Eigen::RealSchur<Eigen::MatrixXd> schur(H);

  const auto& U = schur.matrixU();
  const auto X = U.topRightCorner(n, n);
  const auto M = U.topLeftCorner(n, n);

  return X * M.inverse();
}
}  // namespace ctrl
}  // namespace tobas
