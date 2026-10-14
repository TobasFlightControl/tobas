// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_quadprog/quadprogpp.hpp"

#include <QuadProg++/QuadProg++.hh>

namespace quadprogpp
{
void matrixEigenToQp(const Eigen::MatrixXd& _e, Matrix<double>& _q)
{
  if (_q.nrows() != _e.rows() || _q.ncols() != _e.cols()) {
    _q.resize(_e.rows(), _e.cols());
  }

  for (Eigen::Index i = 0; i < _e.rows(); ++i) {
    for (Eigen::Index j = 0; j < _e.cols(); ++j) {
      _q[i][j] = _e(i, j);
    }
  }
}

void matrixQpToEigen(const Matrix<double>& _q, Eigen::MatrixXd& _e)
{
  // Eigen cannot be resized casually here, so ensure the argument already has the correct size.
  assert(_e.rows() == _q.nrows() && _e.cols() == _q.ncols());

  for (Eigen::Index i = 0; i < _e.rows(); ++i) {
    for (Eigen::Index j = 0; j < _e.cols(); ++j) {
      _e(i, j) = _q[i][j];
    }
  }
}

void vectorEigenToQp(const Eigen::VectorXd& _e, Vector<double>& _q)
{
  assert(_e.cols() == 1);

  if (_q.size() != _e.size()) {
    _q.resize(_e.rows());
  }

  for (Eigen::Index i = 0; i < _e.rows(); ++i) {
    _q[i] = _e(i);
  }
}

void vectorQpToEigen(const Vector<double>& _q, Eigen::VectorXd& _e)
{
  assert(_e.rows() == _q.size());
  assert(_e.cols() == 1);

  for (Eigen::Index i = 0; i < _e.rows(); ++i) {
    _e(i) = _q[i];
  }
}
}  // namespace quadprogpp

namespace tobas
{
namespace quadprog
{
QuadProgppSolver::QuadProgppSolver() : super()
{
}

std::expected<Eigen::VectorXd, std::string> QuadProgppSolver::solve()
{
  checkProblemValidity();

  // Scale the problem.
  const auto scaled = scaleProblem();

  // Convert to QuadProg++ matrices.
  quadprogpp::matrixEigenToQp(scaled.P, G_);
  quadprogpp::vectorEigenToQp(scaled.q, g0_);
  quadprogpp::matrixEigenToQp(-scaled.G.transpose(), CE_);
  quadprogpp::vectorEigenToQp(scaled.h, ce0_);
  quadprogpp::matrixEigenToQp(-scaled.A.transpose(), CI_);
  quadprogpp::vectorEigenToQp(scaled.b, ci0_);

  // Solve the QP.
  const double f_value = quadprogpp::solve_quadprog(G_, g0_, CE_, ce0_, CI_, ci0_, x_);
  if (f_value > 1e+10) {
    return std::unexpected("QPP is infeasible.");
  }

  Eigen::VectorXd x_scaled(x_.size());
  quadprogpp::vectorQpToEigen(x_, x_scaled);

  // Restore the solution to the original scale.
  return x_scaled.cwiseProduct(x_scale).eval();
}
}  // namespace quadprog
}  // namespace tobas
