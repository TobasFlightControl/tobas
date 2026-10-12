// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <functional>
#include <string>

namespace tobas
{
namespace nlp
{
/**
 * One-dimensional Newton method solver.
 * Solve: f(x) = 0
 */
class NewtonSolver1d
{
public:
  explicit NewtonSolver1d();

  void initialize(std::function<double(double)> f, std::function<double(double)> dfdx);

  /** Solve from the initial value x and return the solution or an error message. */
  std::expected<double, std::string> solve(double x);

  void setMaximumIterations(size_t max_iter);
  void setAbsoluteTolerance(double abs_tol);

private:
  std::function<double(double)> f_;
  std::function<double(double)> dfdx_;

  // Configurations
  size_t max_iter_ = 100;
  double abs_tol_ = 1e-10;
};
}  // namespace nlp
}  // namespace tobas
