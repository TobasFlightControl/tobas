// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_quadprog/primal_dual_interior_point.hpp"

#include <cassert>
#include <utility>

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/LU>

#include "tobas_quadprog/dual_active_set.hpp"

namespace tobas::quadprog
{
PrimalDualInteriorPointSolver::PrimalDualInteriorPointSolver()
{
}

std::expected<Eigen::VectorXd, std::string> PrimalDualInteriorPointSolver::solve()
{
  checkProblemValidity();

  // Scaling
  const auto scaled = scaleProblem();

  if (is_first_solve_) {
    if (const auto result = initialize(scaled); !result) {
      return std::unexpected("Failed to initialize decision variables: " + result.error());
    }
    is_first_solve_ = false;
  }

  // If there are no constraints, find the stationary point and finish.
  if (eq_dim_ == 0 && ineq_dim_ == 0) {
    const Eigen::LLT<Eigen::MatrixXd> llt(scaled.P);
    if (llt.info() == Eigen::NumericalIssue) {
      return std::unexpected("Cholesky decomposition failed.");
    }

    theta_ = -llt.solve(scaled.q);
    return theta_.cwiseProduct(x_scale).eval();
  }

  // Iteration
  // Real-time requirements will impose a hard bound on the number of interior-point iterations,
  // hence it is assumed fixed a priori.
  for (size_t _ = 0; _ < num_iter_; ++_) {
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic> W = lam_.cwiseProduct(s_.cwiseInverse()).asDiagonal();
    const double mu = lam_.dot(s_) / static_cast<double>(eq_dim_ + ineq_dim_);  // NaN when unconstrained.
    const Eigen::VectorXd sigma_mu_sinv = sigma_ * mu * s_.cwiseInverse();

    // 1
    A_.topLeftCorner(var_dim_, var_dim_) = scaled.P + scaled.A.transpose() * W * scaled.A;
    A_.topRightCorner(var_dim_, eq_dim_) = scaled.G.transpose();
    A_.bottomLeftCorner(eq_dim_, var_dim_) = scaled.G;

    // 2
    b_.head(var_dim_) = -scaled.q - scaled.A.transpose() * (lam_ - W * scaled.b + sigma_mu_sinv);
    b_.bottomRows(eq_dim_) = -scaled.G * theta_ + scaled.h;

    // 3
    const Eigen::PartialPivLU<Eigen::MatrixXd> lu(A_);
    const Eigen::VectorXd z = lu.solve(b_);  // TODO: Check whether the matrix is regular.
    const Eigen::VectorXd theta_dtheta = z.head(var_dim_);
    // const Eigen::VectorXd dnu = z.bottomRows(eq_dim_);
    const Eigen::VectorXd dtheta = theta_dtheta - theta_;
    const Eigen::VectorXd G_theta_g = scaled.A * theta_dtheta - scaled.b;

    // 4
    const Eigen::VectorXd dlam = W * G_theta_g + sigma_mu_sinv;

    // 5
    const Eigen::VectorXd ds = -s_ - G_theta_g;

    // 6
    const double alpha = findAlpha(dlam, ds);

    // 7
    theta_ += alpha * dtheta;
    // nu_ += alpha * dnu;
    lam_ += alpha * dlam;
    s_ += alpha * ds;
  }

  // TODO: Check solution convergence and feasibility.

  // Restore the solution to the original scale.
  return theta_.cwiseProduct(x_scale).eval();
}

void PrimalDualInteriorPointSolver::setNumberOfIterations(size_t _num_iter)
{
  assert(_num_iter > 0);
  num_iter_ = _num_iter;
}

void PrimalDualInteriorPointSolver::setSigma(double _sigma)
{
  assert(_sigma > 0.0 && _sigma < 1.0);
  sigma_ = _sigma;
}

void PrimalDualInteriorPointSolver::setAlphaTolerance(double _alpha_tol)
{
  assert(_alpha_tol > 0.0 && _alpha_tol < 1.0);
  alpha_tol_ = _alpha_tol;
}

std::expected<void, std::string> PrimalDualInteriorPointSolver::initialize(const QuadProgProblem& _scaled)
{
  var_dim_ = _scaled.q.rows();
  eq_dim_ = _scaled.h.rows();
  ineq_dim_ = _scaled.b.rows();

  // Find a feasible initial solution using the active-set method.
  DualActiveSetSolver active_set_solver_;
  active_set_solver_.problem = _scaled;
  active_set_solver_.x_scale = Eigen::VectorXd::Ones(problem.varSize());
  const auto init_theta = active_set_solver_.solve();
  if (!init_theta) {
    return std::unexpected(init_theta.error());
  }
  theta_ = std::move(*init_theta);

  // Initialize inequality constraint Lagrange multipliers and slack variables to 1.
  lam_ = Eigen::VectorXd::Ones(ineq_dim_);
  s_ = Eigen::VectorXd::Ones(ineq_dim_);

  A_ = Eigen::MatrixXd::Zero(var_dim_ + eq_dim_, var_dim_ + eq_dim_);
  b_ = Eigen::VectorXd::Zero(var_dim_ + eq_dim_);

  return {};
}

double PrimalDualInteriorPointSolver::findAlpha(const Eigen::VectorXd& _dlam, const Eigen::VectorXd& _ds) const
{
  double lb = 0.0;
  double ub = 1.0;

  while (ub - lb > alpha_tol_) {
    const auto mid = (lb + ub) / 2;
    const Eigen::VectorXd lam = lam_ + mid * _dlam;
    const Eigen::VectorXd s = s_ + mid * _ds;
    if ((lam.array() > 0).all() && (s.array() > 0).all()) {
      lb = mid;
    }
    else {
      ub = mid;
    }
  }

  return lb;
}
}  // namespace tobas::quadprog
