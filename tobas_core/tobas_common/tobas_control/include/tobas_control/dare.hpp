// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <eigen3/Eigen/Core>

namespace tobas
{
namespace ctrl
{
/** Solve the discrete-time algebraic Riccati equation. */
std::expected<Eigen::MatrixXd, std::string> dare(
  const Eigen::MatrixXd& _a,
  const Eigen::MatrixXd& _b,
  const Eigen::MatrixXd& _q,
  const Eigen::MatrixXd& _r,
  double _tol = 1e-3,
  size_t _max_iter = 10000) noexcept;
}  // namespace ctrl
}  // namespace tobas
