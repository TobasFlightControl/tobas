// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

namespace tobas
{
namespace ctrl
{
/**
 * @brief Solve the discrete-time algebraic Riccati equation.
 */
Eigen::MatrixXd dare(
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& B,
  const Eigen::MatrixXd& Q,
  const Eigen::MatrixXd& R,
  const double& tol = 1e-3,
  size_t max_iter = 10000);
}  // namespace ctrl
}  // namespace tobas
