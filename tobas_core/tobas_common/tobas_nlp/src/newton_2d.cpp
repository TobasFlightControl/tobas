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
  std::function<Eigen::Vector2d(const Eigen::Vector2d&)> f,
  std::function<Eigen::Matrix2d(const Eigen::Vector2d&)> dfdx)
{
  f_ = f;
  dfdx_ = dfdx;
}

NewtonSolver2d::Error NewtonSolver2d::solve(Eigen::Vector2d& x)
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

    assert(std::isfinite(f.norm()));
    assert(std::isfinite(dfdx.norm()));

    Eigen::FullPivLU<Eigen::Matrix2d> lu(dfdx);
    // Stationary point case.
    if (!lu.isInvertible()) {
      if (f.norm() < std::numeric_limits<double>::epsilon()) {
        // Exit if already at the solution.
        return error_code_ = kNoError;
      }
      else {
        // Return an error if not converged.
        return error_code_ = kInfeasible;
      }
    }

    const auto dx = -lu.solve(f);
    x += dx;

    // Termination check.
    if (dx.norm() < abs_tol_) {
      return error_code_ = kNoError;
    }
  }

  return error_code_ = kMaxIterationExceeded;
}

NewtonSolver2d::Error NewtonSolver2d::errorCode() const
{
  return error_code_;
}

const char* NewtonSolver2d::errorMessage() const
{
  switch (error_code_) {
    case kNoError:
      return "No error.";
    case kMaxIterationExceeded:
      return "The number of iterations exceeded the limit.";
    case kInfeasible:
      return "The algorithm is infeasible.";
    default:
      return "Unknown error.";
  }
}

bool NewtonSolver2d::setMaximumIterations(size_t max_iter)
{
  max_iter_ = max_iter;
  return true;
}

bool NewtonSolver2d::setAbsoluteTolerance(double abs_tol)
{
  if (abs_tol <= 0.0) {
    std::cerr << "Absolute tolerance must be positive." << std::endl;
    return false;
  }

  abs_tol_ = abs_tol;
  return true;
}
}  // namespace nlp
}  // namespace tobas
