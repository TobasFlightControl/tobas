// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

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
  enum Error
  {
    kNoError = 0,
    kMaxIterationExceeded = -1,
    kInfeasible = -2,
  };

  explicit NewtonSolver2d();

  void initialize(std::function<Eigen::Vector2d(const Eigen::Vector2d&)> f, std::function<Eigen::Matrix2d(const Eigen::Vector2d&)> dfdx);

  Error solve(Eigen::Vector2d& x);

  Error errorCode() const;
  const char* errorMessage() const;

  bool setMaximumIterations(size_t max_iter);
  bool setAbsoluteTolerance(double abs_tol);

private:
  Error error_code_;

  std::function<Eigen::Vector2d(Eigen::Vector2d)> f_;
  std::function<Eigen::Matrix2d(Eigen::Vector2d)> dfdx_;

  // Configurations
  size_t max_iter_ = 100;
  double abs_tol_ = 1e-10;
};
}  // namespace nlp
}  // namespace tobas
