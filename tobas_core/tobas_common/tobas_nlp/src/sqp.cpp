// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_nlp/sqp.hpp"

#include <cassert>

#include <tobas_eigen_tools/linalg.hpp>

// #define TRACE_SOLVER

namespace tobas
{
namespace nlp
{
SQP::SQP()
{
}

void SQP::initialize(
  const Eigen::VectorXd& x0,
  std::function<double(const Eigen::VectorXd&)> f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> g,
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h,
  std::function<Eigen::RowVectorXd(const Eigen::VectorXd&)> dfdx,
  std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> dgdx,
  std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> dhdx,
  std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> dFdx,
  std::function<Eigen::Tensor3Xd(const Eigen::VectorXd&)> dGdx,
  std::function<Eigen::Tensor3Xd(const Eigen::VectorXd&)> dHdx)
{
  n_ = x0.size();
  m_ = g(x0).size();
  p_ = h(x0).size();

  x_ = x0;
  lam_ = Eigen::VectorXd::Zero(m_);
  mu_ = Eigen::VectorXd::Zero(p_);

  f_ = f;
  g_ = g;
  h_ = h;
  dfdx_ = dfdx;
  dgdx_ = dgdx;
  dhdx_ = dhdx;
  dFdx_ = dFdx;
  dGdx_ = dGdx;
  dHdx_ = dHdx;

  if (qp_.x_scale.size() != n_) {
    qp_.x_scale = Eigen::VectorXd::Ones(n_);
  }
}

std::expected<Eigen::VectorXd, std::string> SQP::solve()
{
  iter_ = 0;

  while (true) {
    // Check the iteration limit.
    if (max_iter_ > 0 && ++iter_ > max_iter_) {
      return std::unexpected("The number of iterations exceeded the limit.");
    }

    // Calculate the Hessian matrix of the Lagrangian.
    auto H = dFdx_(x_);
    if (m_ > 0) {
      H += lam_.transpose().eval() * dGdx_(x_);
    }
    if (p_ > 0) {
      H += mu_.transpose().eval() * dHdx_(x_);
    }

    // Solve the local QP.
    qp_.problem.P = eigen::nearestPositiveDefinite(H, 1e-6);
    qp_.problem.q = dfdx_(x_).transpose();
    qp_.problem.A = dgdx_(x_);
    qp_.problem.b = -g_(x_);
    qp_.problem.G = dhdx_(x_);
    qp_.problem.h = -h_(x_);

    const auto dx = qp_.solve();
    if (!dx) {
      return dx;
    }

    // Update optimization variables.
    x_ += *dx;
    lam_ = qp_.getLagrangeMultipliersIneq();
    mu_ = qp_.getLagrangeMultipliersEq();

#ifdef TRACE_SOLVER
    std::cout << "Iteration: " << iter_ << std::endl;
    std::cout << "x = " << x_.transpose() << std::endl;
    std::cout << "lambda = " << lam_.transpose() << std::endl;
    std::cout << "mu = " << mu_.transpose() << std::endl;
    std::cout << "----------" << std::endl;
#endif

    // Termination check.
    // cf. https://kotakku.github.io/cpp_robotics/tech_note/optimize/tolerances_and_stopping/
    if ((dx->cwiseAbs().array() < (rel_tol_ * qp_.x_scale).array()).all()) {
      return x_;
    }
  }
}

size_t SQP::iterations() const
{
  return iter_;
}

void SQP::setMaximumIterations(size_t max_iter)
{
  max_iter_ = max_iter;
}

void SQP::setRelativeTolerance(double rel_tol)
{
  assert(rel_tol > 0.0);
  rel_tol_ = rel_tol;
}

void SQP::setVariableScales(const Eigen::VectorXd& x_scale)
{
  assert(x_scale.size() == n_);
  assert((x_scale.array() > 0.0).all());
  qp_.x_scale = x_scale;
}
}  // namespace nlp
}  // namespace tobas
