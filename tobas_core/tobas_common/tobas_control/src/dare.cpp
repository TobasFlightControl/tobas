// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/dare.hpp"

#include <iostream>

#include <tobas_eigen_tools/linalg.hpp>

#include "tobas_control/util.hpp"

namespace tobas
{
namespace ctrl
{
std::expected<Eigen::MatrixXd, std::string> dare(
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& B,
  const Eigen::MatrixXd& Q,
  const Eigen::MatrixXd& R,
  const double& tol,
  size_t max_iter) noexcept
{
  const auto n = A.rows();
  [[maybe_unused]] const auto l = B.cols();

  assert(A.cols() == n);
  assert(B.rows() == n);
  assert(Q.rows() == n && Q.cols() == n);
  assert(R.rows() == l && R.rows() == l);
  assert(isControllable(A, B));
  assert(eigen::isSymmetricSemiPositiveDefinite(Q));
  assert(eigen::isSymmetricPositiveDefinite(R));
  assert(tol > 0.0);

  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
  Eigen::MatrixXd X_prev = Eigen::MatrixXd::Zero(n, n);
  Eigen::MatrixXd X_next = Eigen::MatrixXd::Identity(n, n);
  size_t iter = 0;

  while ((X_next - X_prev).norm() / X_next.norm() > tol) {
    X_prev = X_next;

    // Prior estimate.
    const Eigen::MatrixXd X_mid = A.transpose() * X_prev.selfadjointView<Eigen::Lower>() * A + Q;

    // Posterior estimate.
    const Eigen::MatrixXd XB = X_mid.selfadjointView<Eigen::Lower>() * B;
    const auto G = XB * (B.transpose() * XB + R).inverse();
    const auto I_GBt = I - G * B.transpose();
    X_next = I_GBt * X_mid.selfadjointView<Eigen::Lower>();

    if (iter++ > max_iter) {
      return std::unexpected("DARE failed to converge in " + std::to_string(max_iter) + " iterations.");
    }
  }

  std::cout << "DARE has successfully converged in " << iter << " iterations." << std::endl;
  return X_next;
}
}  // namespace ctrl
}  // namespace tobas
