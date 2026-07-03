// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

namespace tobas
{
namespace ctrl
{
/**
 * @brief Solve the continuous-time algebraic Riccati equation.
 * cf. Arimoto-Potter method: https://qiita.com/trgkpc/items/8210927d5b035912a153
 */
Eigen::MatrixXd care_ArimotoPotter(
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& B,
  const Eigen::MatrixXd& Q,
  const Eigen::MatrixXd& R);

/**
 * @brief Solve the continuous-time algebraic Riccati equation.
 *
 * @note Does not work correctly (2023/5/24).
 */
Eigen::MatrixXd
care_Schur(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
}  // namespace ctrl
}  // namespace tobas
