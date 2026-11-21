// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_nlp/newton_2d.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

#include <eigen3/Eigen/LU>

// #define TRACE_SOLVER

namespace tobas
{
namespace nlp
{
NewtonSolver2d::NewtonSolver2d()
{
}

void NewtonSolver2d::initialize(
  std::function<Eigen::Vector2d(const Eigen::Vector2d&)> _f,
  std::function<Eigen::Matrix2d(const Eigen::Vector2d&)> _dfdx)
{
  f_ = _f;
  dfdx_ = _dfdx;
}

std::expected<Eigen::Vector2d, std::string> NewtonSolver2d::solve(Eigen::Vector2d& _x)
{
  for (size_t iter = 0; iter < max_iter_; ++iter) {
    const auto f = f_(_x);
    const auto dfdx = dfdx_(_x);

#ifdef TRACE_SOLVER
    std::cout << "Iteration: " << iter << std::endl;
    std::cout << "x = " << x << std::endl;
    std::cout << "f(x) = " << f << std::endl;
    std::cout << "df/dx(x) = " << dfdx << std::endl;
    std::cout << "----------" << std::endl;
#endif

    assert(std::isfinite(f.norm()));
    assert(std::isfinite(dfdx.norm()));

    Eigen::FullPivLU<Eigen::Matrix2d> lu(dfdx);
    // Stationary point case.
    if (!lu.isInvertible()) {
      if (f.norm() < std::numeric_limits<double>::epsilon()) {
        // Exit if already at the solution.
        return _x;
      }
      else {
        // Return an error if not converged.
        return std::unexpected("The algorithm is infeasible.");
      }
    }

    const auto dx = -lu.solve(f);
    _x += dx;

    // Termination check.
    if (dx.norm() < abs_tol_) {
      return _x;
    }
  }

  return std::unexpected("The number of iterations exceeded the limit.");
}

void NewtonSolver2d::setMaximumIterations(size_t _max_iter)
{
  max_iter_ = _max_iter;
}

void NewtonSolver2d::setAbsoluteTolerance(double _abs_tol)
{
  assert(_abs_tol > 0.0);

  abs_tol_ = _abs_tol;
}
}  // namespace nlp
}  // namespace tobas
