// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <functional>

#include <eigen3/Eigen/Core>

namespace tobas
{
namespace nlp
{
/**
 * @brief Two-dimensional Newton method solver.
 *
 * Solve: f(x) = 0
 */
class NewtonSolver2d
{
public:
  explicit NewtonSolver2d();

  void initialize(
    std::function<Eigen::Vector2d(const Eigen::Vector2d&)> _f,
    std::function<Eigen::Matrix2d(const Eigen::Vector2d&)> _dfdx);

  std::expected<Eigen::Vector2d, std::string> solve(Eigen::Vector2d& _x);

  void setMaximumIterations(size_t _max_iter);
  void setAbsoluteTolerance(double _abs_tol);

private:
  std::function<Eigen::Vector2d(Eigen::Vector2d)> f_;
  std::function<Eigen::Matrix2d(Eigen::Vector2d)> dfdx_;

  // Configurations
  size_t max_iter_ = 100;
  double abs_tol_ = 1e-10;
};
}  // namespace nlp
}  // namespace tobas
