#include "tobas_nlp/newton_1d.hpp"

#include <cmath>
#include <iostream>

// #define TRACE_SOLVER

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

NewtonSolver1d::ErrorCode NewtonSolver1d::solve(double& x)
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

    // 停留点の場合
    if (dfdx == 0.) {
      if (f == 0.) {
        // すでに解にいるならば終了
        return error_code_ = kNoError;
      }
      else {
        // 収束していないならばエラー
        return error_code_ = kInfeasible;
      }
    }

    const auto dx = -f / dfdx;
    x += dx;

    // 終了判定
    if (fabs(dx) < abs_tol_) {
      return error_code_ = kNoError;
    }
  }

  return error_code_ = kMaxIterationExceeded;
}

NewtonSolver1d::ErrorCode NewtonSolver1d::errorCode() const
{
  return error_code_;
}

const char* NewtonSolver1d::errorMessage() const
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

bool NewtonSolver1d::setMaximumIterations(size_t max_iter)
{
  max_iter_ = max_iter;
  return true;
}

bool NewtonSolver1d::setAbsoluteTolerance(double abs_tol)
{
  if (abs_tol <= 0.) {
    std::cerr << "Absolute tolerance must be positive." << std::endl;
    return false;
  }

  abs_tol_ = abs_tol;
  return true;
}
}  // namespace nlp
