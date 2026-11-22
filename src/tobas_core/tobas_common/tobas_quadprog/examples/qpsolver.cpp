// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstdlib>
#include <iostream>

#include <eigen3/Eigen/Core>

#include <tobas_quadprog/dual_active_set.hpp>
#include <tobas_quadprog/primal_dual_interior_point.hpp>

int main()
{
  tobas::quadprog::QuadProgProblem problem(2, 0, 2);
  problem.P << 1.0, 0.0, 0.0, 0.5;
  problem.q << 1.5, 1.0;
  problem.A << 1.0, 1.0, -1.0, -1.0;
  problem.b << 1.0, 0.0;
  const Eigen::Vector2d x_scale = Eigen::Vector2d::Ones();

  tobas::quadprog::DualActiveSetSolver das;
  das.problem = problem;
  das.x_scale = x_scale;
  const auto das_result = das.solve();
  if (!das_result) {
    std::cerr << "DualActiveSetSolver failed: " << das_result.error() << '\n';
    return EXIT_FAILURE;
  }
  std::cout << "DualActiveSet solution: " << das_result->transpose() << '\n';

  tobas::quadprog::PrimalDualInteriorPointSolver ipm;
  ipm.problem = problem;
  ipm.x_scale = x_scale;
  const auto ipm_result = ipm.solve();
  if (!ipm_result) {
    std::cerr << "PrimalDualInteriorPointSolver failed: " << ipm_result.error() << '\n';
    return EXIT_FAILURE;
  }
  std::cout << "PrimalDualInteriorPointSolver solution: " << ipm_result->transpose() << '\n';

  return EXIT_SUCCESS;
}
