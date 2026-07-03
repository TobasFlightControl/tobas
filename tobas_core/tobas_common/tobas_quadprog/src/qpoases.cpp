// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_quadprog/qpoases.hpp"

#include <qpOASES.hpp>

#include <tobas_eigen_tools/core.hpp>
#include <tobas_math/core.hpp>

using namespace Eigen;

namespace tobas
{
namespace quadprog
{
QpOasesSolver::QpOasesSolver()
{
}

bool QpOasesSolver::solve()
{
  checkProblemValidity();

  // Scale the problem.
  const auto scaled = scaleProblem();

  // Create matrices for qpOASES.
  const auto var_size = scaled.varSize();
  const auto con_size = scaled.eqSize() + scaled.ineqSize();

  double H[math::sqr(var_size)];
  double g[var_size];
  double A[con_size * var_size];
  double lb[var_size];
  double ub[var_size];
  double lbA[con_size];
  double ubA[con_size];

  std::memcpy(H, scaled.P.data(), sizeof(H));  // Since `H` is symmetric, either column-major or row-major copy works.
  std::memcpy(g, scaled.q.data(), sizeof(g));

  // Copy elements one by one to account for column-major storage.
  const MatrixXd A_eigen = eigen::concat(scaled.G, scaled.A, 0);
  for (Index r = 0; r < con_size; ++r) {
    for (Index c = 0; c < var_size; ++c) {
      A[r * var_size + c] = A_eigen(r, c);
    }
  }

  for (Index i = 0; i < var_size; ++i) {
    lb[i] = -qpOASES::INFTY;
    ub[i] = qpOASES::INFTY;
  }

  const VectorXd inf = VectorXd::Constant(scaled.ineqSize(), -qpOASES::INFTY);
  const VectorXd lbA_eigen = eigen::concat(scaled.h, inf, 0);
  std::memcpy(lbA, lbA_eigen.data(), sizeof(lbA));

  const VectorXd ubA_eigen = eigen::concat(scaled.h, scaled.b, 0);
  std::memcpy(ubA, ubA_eigen.data(), sizeof(ubA));

  // Create the QP solver.
  qpOASES::QProblem solver(var_size, con_size);

  // Configure the QP solver.
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = scaled.eqSize() > 0 ? qpOASES::BT_TRUE : qpOASES::BT_FALSE;
  solver.setOptions(options);

  // Solve the QP.
  solver.init(H, g, A, lb, ub, lbA, ubA, nWSR_);

  double x_opt[var_size];
  const auto ret = solver.getPrimalSolution(x_opt);
  if (ret != qpOASES::SUCCESSFUL_RETURN) {
    error_msg_ = "qpOASES finished with error code " + std::to_string(ret);
    return false;
  }

  // Restore the solution to the original scale.
  VectorXd x_scaled = Map<VectorXd>(x_opt, var_size);
  x_opt_ = x_scaled.cwiseProduct(x_scale);

  return true;
}
}  // namespace quadprog
}  // namespace tobas
