// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_quadprog/dual_active_set.hpp>
#include <tobas_quadprog/primal_dual_interior_point.hpp>
#include <tobas_quadprog/qpoases.hpp>
#include <tobas_quadprog/quadprogpp.hpp>

using namespace std;
using namespace Eigen;

int main()
{
  tobas::quadprog::QuadProgProblem problem(2, 0, 2);
  problem.P << 1.0, 0.0, 0.0, 0.5;
  problem.q << 1.5, 1.0;
  problem.A << 1.0, 1.0, -1.0, -1.0;
  problem.b << 1.0, 0.0;
  const Vector2d x_scale = Vector2d::Ones();

  tobas::quadprog::QuadProgppSolver quadprog;
  quadprog.problem = problem;
  quadprog.x_scale = x_scale;
  const auto quadprog_result = quadprog.solve();
  if (!quadprog_result) {
    cerr << "QuadProgppSolver failed: " << quadprog_result.error() << endl;
    return EXIT_FAILURE;
  }
  cout << "QuadProg++ solution: " << quadprog_result->transpose() << endl;

  tobas::quadprog::QpOasesSolver qpoases;
  qpoases.problem = problem;
  qpoases.x_scale = x_scale;
  const auto qpoases_result = qpoases.solve();
  if (!qpoases_result) {
    cerr << "QpOasesSolver failed: " << qpoases_result.error() << endl;
    return EXIT_FAILURE;
  }
  cout << "qpOASES solution: " << qpoases_result->transpose() << endl;

  tobas::quadprog::DualActiveSetSolver das;
  das.problem = problem;
  das.x_scale = x_scale;
  const auto das_result = das.solve();
  if (!das_result) {
    cerr << "DualActiveSetSolver failed: " << das_result.error() << endl;
    return EXIT_FAILURE;
  }
  cout << "DualActiveSet solution: " << das_result->transpose() << endl;

  tobas::quadprog::PrimalDualInteriorPointSolver ipm;
  ipm.problem = problem;
  ipm.x_scale = x_scale;
  const auto ipm_result = ipm.solve();
  if (!ipm_result) {
    cerr << "PrimalDualInteriorPointSolver failed: " << ipm_result.error() << endl;
    return EXIT_FAILURE;
  }
  cout << "PrimalDualInteriorPointSolver solution: " << ipm_result->transpose() << endl;

  return EXIT_SUCCESS;
}
