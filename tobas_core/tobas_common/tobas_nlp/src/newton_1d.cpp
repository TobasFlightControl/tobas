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

void NewtonSolver1d::initialize(std::function<double(double)> _f, std::function<double(double)> _dfdx)
{
  f_ = _f;
  dfdx_ = _dfdx;
}

std::expected<double, std::string> NewtonSolver1d::solve(double _x)
{
  for (size_t iter = 0; iter < max_iter_; ++iter) {
    const auto f = f_(_x);
    const auto dfdx = dfdx_(_x);

#ifdef TRACE_SOLVER
    std::cout << "Iteration: " << iter << std::endl;
    std::cout << "x = " << _x << std::endl;
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
        return _x;
      }
      else {
        // Return an error if not converged.
        return std::unexpected("The algorithm is infeasible.");
      }
    }

    const auto dx = -f / dfdx;
    _x += dx;

    // Termination check.
    if (std::abs(dx) < abs_tol_) {
      return _x;
    }
  }

  return std::unexpected("The number of iterations exceeded the limit.");
}

void NewtonSolver1d::setMaximumIterations(size_t _max_iter)
{
  max_iter_ = _max_iter;
}

void NewtonSolver1d::setAbsoluteTolerance(double _abs_tol)
{
  assert(_abs_tol > 0.0);

  abs_tol_ = _abs_tol;
}
}  // namespace nlp
}  // namespace tobas
