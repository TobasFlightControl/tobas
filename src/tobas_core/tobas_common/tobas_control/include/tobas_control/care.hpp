// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <eigen3/Eigen/Core>

namespace tobas::ctrl
{
/**
 * Solve the continuous-time algebraic Riccati equation.
 *
 * @see Arimoto-Potter method: https://qiita.com/trgkpc/items/8210927d5b035912a153
 */
std::expected<Eigen::MatrixXd, std::string> care_ArimotoPotter(
  const Eigen::MatrixXd& _a,
  const Eigen::MatrixXd& _b,
  const Eigen::MatrixXd& _q,
  const Eigen::MatrixXd& _r) noexcept;

/**
 * Solve the continuous-time algebraic Riccati equation.
 *
 * @note Does not work correctly (2023/5/24).
 */
Eigen::MatrixXd
care_Schur(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _b, const Eigen::MatrixXd& _q, const Eigen::MatrixXd& _r);
}  // namespace tobas::ctrl
