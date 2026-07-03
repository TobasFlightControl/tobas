// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

namespace tobas
{
namespace quadprog
{
/**
 * @brief Create equivalent matrix inequalities (`A @ x <= b`) from variable ranges (`lb <= x <= ub`).
 *
 * @param lb Lower bounds.
 * @param ub Upper bounds.
 * @param inf Values greater than this are omitted from the matrix inequalities.
 *
 * @return The (`A`, `b`) pair for `A @ x <= b`.
 */
void matIneqFromRange(
  const Eigen::VectorXd& lb,
  const Eigen::VectorXd& ub,
  Eigen::MatrixXd& A,
  Eigen::VectorXd& b,
  const double inf = 1E+12);
}  // namespace quadprog
}  // namespace tobas
