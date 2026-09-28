// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <functional>

namespace tobas
{
namespace nlp
{
/**
 * @brief One-dimensional Newton method solver.
 *
 * Solve: f(x) = 0
 */
class NewtonSolver1d
{
public:
  enum Error
  {
    kNoError = 0,
    kMaxIterationExceeded = -1,
    kInfeasible = -2,
  };

  explicit NewtonSolver1d();

  void initialize(std::function<double(double)> f, std::function<double(double)> dfdx);

  Error solve(double& x);

  Error errorCode() const;
  const char* errorMessage() const;

  bool setMaximumIterations(size_t max_iter);
  bool setAbsoluteTolerance(double abs_tol);

private:
  Error error_code_;

  std::function<double(double)> f_;
  std::function<double(double)> dfdx_;

  // Configurations
  size_t max_iter_ = 100;
  double abs_tol_ = 1e-10;
};
}  // namespace nlp
}  // namespace tobas
