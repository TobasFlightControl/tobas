// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_nlp/newton_1d.hpp"

#include <cassert>
#include <cmath>

// #define TRACE_SOLVER

namespace tobas
{
namespace nlp
{
NewtonSolver1d::NewtonSolver1d()
{
}

void NewtonSolver1d::initialize(std::function<double(double)> f, std::function<double(double)> dfdx)
{
  f_ = f;
  dfdx_ = dfdx;
}

std::expected<double, std::string> NewtonSolver1d::solve(double x)
{
  for (size_t iter = 0; iter < max_iter_; ++iter) {
    const auto f = f_(x);
    const auto dfdx = dfdx_(x);

#ifdef TRACE_SOLVER
    std::cout << "Iteration: " << iter << std::endl;
    std::cout << "x = " << x << std::endl;
    std::cout << "f(x) = " << f << std::endl;
    std::cout << "df/dx(x) = " << dfdx << std::endl;
    std::cout << "----------" << std::endl;
#endif

    assert(std::isfinite(f));
    assert(std::isfinite(dfdx));

    // Stationary point case.
    constexpr auto kEps = std::numeric_limits<double>::epsilon();
    if (dfdx < kEps) {
      if (f < kEps) {
        // Exit if already at the solution.
        return x;
      }
      else {
        // Return an error if not converged.
        return std::unexpected("The algorithm is infeasible.");
      }
    }

    const auto dx = -f / dfdx;
    x += dx;

    // Termination check.
    if (std::abs(dx) < abs_tol_) {
      return x;
    }
  }

  return std::unexpected("The number of iterations exceeded the limit.");
}

void NewtonSolver1d::setMaximumIterations(size_t max_iter)
{
  max_iter_ = max_iter;
}

void NewtonSolver1d::setAbsoluteTolerance(double abs_tol)
{
  assert(abs_tol > 0.0);

  abs_tol_ = abs_tol;
}
}  // namespace nlp
}  // namespace tobas
