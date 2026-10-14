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
  const Eigen::MatrixXd& _a,
  const Eigen::MatrixXd& _b,
  const Eigen::MatrixXd& _q,
  const Eigen::MatrixXd& _r,
  double _tol,
  size_t _max_iter) noexcept
{
  const auto n = _a.rows();
  [[maybe_unused]] const auto l = _b.cols();

  assert(_a.cols() == n);
  assert(_b.rows() == n);
  assert(_q.rows() == n && _q.cols() == n);
  assert(_r.rows() == l && _r.rows() == l);
  assert(isControllable(_a, _b));
  assert(eigen::isSymmetricSemiPositiveDefinite(_q));
  assert(eigen::isSymmetricPositiveDefinite(_r));
  assert(_tol > 0.0);

  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
  Eigen::MatrixXd X_prev = Eigen::MatrixXd::Zero(n, n);
  Eigen::MatrixXd X_next = Eigen::MatrixXd::Identity(n, n);
  size_t iter = 0;

  while ((X_next - X_prev).norm() / X_next.norm() > _tol) {
    X_prev = X_next;

    // Prior estimate.
    const Eigen::MatrixXd X_mid = _a.transpose() * X_prev.selfadjointView<Eigen::Lower>() * _a + _q;

    // Posterior estimate.
    const Eigen::MatrixXd XB = X_mid.selfadjointView<Eigen::Lower>() * _b;
    const auto G = XB * (_b.transpose() * XB + _r).inverse();
    const auto I_GBt = I - G * _b.transpose();
    X_next = I_GBt * X_mid.selfadjointView<Eigen::Lower>();

    if (iter++ > _max_iter) {
      return std::unexpected("DARE failed to converge in " + std::to_string(_max_iter) + " iterations.");
    }
  }

  std::cout << "DARE has successfully converged in " << iter << " iterations." << std::endl;
  return X_next;
}
}  // namespace ctrl
}  // namespace tobas
