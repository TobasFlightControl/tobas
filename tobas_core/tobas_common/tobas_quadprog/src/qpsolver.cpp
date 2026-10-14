// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_quadprog/qpsolver.hpp"

#include <tobas_eigen_tools/core.hpp>

namespace tobas::quadprog
{
QuadProgProblem::QuadProgProblem(Eigen::Index _var_size, Eigen::Index _eq_size, Eigen::Index _ineq_size)
{
  resize(_var_size, _eq_size, _ineq_size);
}

QuadProgProblem::QuadProgProblem()
{
}

void QuadProgProblem::resize(Eigen::Index _var_size, Eigen::Index _eq_size, Eigen::Index _ineq_size)
{
  P.conservativeResize(_var_size, _var_size);
  q.conservativeResize(_var_size);
  G.conservativeResize(_eq_size, _var_size);
  h.conservativeResize(_eq_size);
  A.conservativeResize(_ineq_size, _var_size);
  b.conservativeResize(_ineq_size);
}

void QuadProgProblem::setZero()
{
  P.setZero();
  q.setZero();
  G.setZero();
  h.setZero();
  A.setZero();
  b.setZero();
}

bool QuadProgProblem::isSizeMatch() const
{
  bool res = true;

  res &= P.rows() == varSize();
  res &= P.cols() == varSize();
  res &= q.size() == varSize();
  res &= G.rows() == eqSize();
  res &= G.cols() == varSize();
  res &= h.size() == eqSize();
  res &= A.rows() == ineqSize();
  res &= A.cols() == varSize();
  res &= b.size() == ineqSize();

  return res;
}

bool QuadProgProblem::isFinite() const
{
  return eigen::isFinite(P) && eigen::isFinite(q) && eigen::isFinite(G) && eigen::isFinite(h) && eigen::isFinite(A) &&
         eigen::isFinite(b);
}

std::ostream& operator<<(std::ostream& _os, const QuadProgProblem& _arg)
{
  _os << "P:\n" << _arg.P << '\n';
  _os << "q:\n" << _arg.q << '\n';
  _os << "G:\n" << _arg.G << '\n';
  _os << "h:\n" << _arg.h << '\n';
  _os << "A:\n" << _arg.A << '\n';
  _os << "b:\n" << _arg.b << '\n';
  return _os;
}

QuadProgSolver::QuadProgSolver()
{
}

void QuadProgSolver::resize(Eigen::Index _var_size, Eigen::Index _eq_size, Eigen::Index _ineq_size)
{
  problem.resize(_var_size, _eq_size, _ineq_size);
  x_scale.conservativeResize(_var_size);
}

void QuadProgSolver::setZero()
{
  problem.setZero();
  x_scale.setZero();
}

std::ostream& operator<<(std::ostream& _os, const QuadProgSolver& _arg)
{
  _os << "problem:\n" << _arg.problem << '\n';
  _os << "x_scale:\n" << _arg.x_scale.transpose() << '\n';
  return _os;
}

QuadProgProblem QuadProgSolver::scaleProblem() const
{
  QuadProgProblem scaled;

  // Scale so each element of `x` has a similar absolute value (memo: 2-21).
  // This changes the optimization variables but not the adjoint variables.
  const Eigen::DiagonalMatrix<double, Eigen::Dynamic> x_scale_diag = x_scale.asDiagonal();
  scaled.P = x_scale_diag * problem.P * x_scale_diag;
  scaled.q = x_scale_diag * problem.q;
  scaled.G = problem.G * x_scale_diag;
  scaled.h = problem.h;
  scaled.A = problem.A * x_scale_diag;
  scaled.b = problem.b;

  // Scale so the maximum diagonal element of `P` becomes 1.
  // This only multiplies the Lagrangian by a constant, so neither optimization variables nor adjoint variables change.
  const auto P_diag_max = scaled.P.diagonal().maxCoeff();
  assert(P_diag_max > 0);
  scaled.P /= P_diag_max;
  scaled.q /= P_diag_max;
  scaled.G /= P_diag_max;
  scaled.h /= P_diag_max;
  scaled.A /= P_diag_max;
  scaled.b /= P_diag_max;

  // TODO: Scale each constraint so the maximum absolute coefficient becomes 1.
  // Note that this changes the adjoint variables.

  return scaled;
}

void QuadProgSolver::checkProblemValidity() const
{
  assert(problem.isSizeMatch());
  assert(problem.isFinite());
  assert(x_scale.size() == problem.varSize());
  assert((x_scale.array() > 0).all());
}
}  // namespace tobas::quadprog
