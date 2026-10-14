// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

namespace tobas::quadprog
{
/**
 * Create equivalent matrix inequalities (`A @ x <= b`) from variable ranges (`lb <= x <= ub`).
 *
 * @param _lb Lower bounds.
 * @param _ub Upper bounds.
 * @param _inf Values greater than this are omitted from the matrix inequalities.
 * @return The (`A`, `b`) pair for `A @ x <= b`.
 */
void matIneqFromRange(
  const Eigen::VectorXd& _lb,
  const Eigen::VectorXd& _ub,
  Eigen::MatrixXd& _a,
  Eigen::VectorXd& _b,
  double _inf = 1E+12);
}  // namespace tobas::quadprog
