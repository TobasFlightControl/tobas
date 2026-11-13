// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/care.hpp"

#include <eigen3/Eigen/LU>

#include <tobas_eigen_tools/linalg.hpp>

#include "tobas_control/util.hpp"

namespace tobas::ctrl
{
std::expected<Eigen::MatrixXd, std::string> care_ArimotoPotter(
  const Eigen::MatrixXd& _a,
  const Eigen::MatrixXd& _b,
  const Eigen::MatrixXd& _q,
  const Eigen::MatrixXd& _r) noexcept
{
  const auto n = _a.rows();
  [[maybe_unused]] const auto l = _b.cols();

  assert(_a.rows() == n && _a.cols() == n);
  assert(eigen::isFinite(_a));

  assert(_b.rows() == n && _b.cols() == l);
  assert(eigen::isFinite(_b));

  assert(_q.rows() == n && _q.cols() == n);
  assert(eigen::isFinite(_q));
  assert(eigen::isSemiPositiveDefinite(_q));

  assert(_r.rows() == l && _r.cols() == l);
  assert(eigen::isFinite(_r));
  assert(eigen::isPositiveDefinite(_r));

  // TODO: Check stabilizability.

  // Make Hamilton matrix.
  Eigen::MatrixXd H(n * 2, n * 2);
  H.topLeftCorner(n, n) = _a;
  H.topRightCorner(n, n) = -_b * _r.inverse() * _b.transpose();
  H.bottomLeftCorner(n, n) = -_q;
  H.bottomRightCorner(n, n) = -_a.transpose();

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
care_Schur(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _b, const Eigen::MatrixXd& _q, const Eigen::MatrixXd& _r)
{
  const auto n = _a.rows();
  [[maybe_unused]] const auto l = _b.cols();

  assert(_a.rows() == n && _a.cols() == n);
  assert(_b.rows() == n && _b.cols() == l);
  assert(_q.rows() == n && _q.cols() == n);
  assert(_r.rows() == l && _r.cols() == l);

  assert(eigen::isFinite(_a));
  assert(eigen::isFinite(_b));
  assert(eigen::isFinite(_q));
  assert(eigen::isFinite(_r));

  assert(isControllable(_a, _b));
  assert(eigen::isSemiPositiveDefinite(_q));
  assert(eigen::isPositiveDefinite(_r));

  Eigen::MatrixXd H(n * 2, n * 2);
  H.topLeftCorner(n, n) = _a;
  H.topRightCorner(n, n) = -_b * _r.inverse() * _b.transpose();
  H.bottomLeftCorner(n, n) = -_q;
  H.bottomRightCorner(n, n) = -_a.transpose();

  const Eigen::RealSchur<Eigen::MatrixXd> schur(H);

  const auto& U = schur.matrixU();
  const auto X = U.topRightCorner(n, n);
  const auto M = U.topLeftCorner(n, n);

  return X * M.inverse();
}
}  // namespace tobas::ctrl
