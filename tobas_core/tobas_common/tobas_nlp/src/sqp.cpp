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
  const Eigen::VectorXd& _x0,
  std::function<double(const Eigen::VectorXd&)> _f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> _g,
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> _h,
  std::function<Eigen::RowVectorXd(const Eigen::VectorXd&)> _dfdx,
  std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> _dgdx,
  std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> _dhdx,
  std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> _d_fdx,
  std::function<Eigen::Tensor3Xd(const Eigen::VectorXd&)> _d_gdx,
  std::function<Eigen::Tensor3Xd(const Eigen::VectorXd&)> _d_hdx)
{
  n_ = _x0.size();
  m_ = _g(_x0).size();
  p_ = _h(_x0).size();

  x_ = _x0;
  lam_ = Eigen::VectorXd::Zero(m_);
  mu_ = Eigen::VectorXd::Zero(p_);

  f_ = _f;
  g_ = _g;
  h_ = _h;
  dfdx_ = _dfdx;
  dgdx_ = _dgdx;
  dhdx_ = _dhdx;
  dFdx_ = _d_fdx;
  dGdx_ = _d_gdx;
  dHdx_ = _d_hdx;

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
    // Ref: https://kotakku.github.io/cpp_robotics/tech_note/optimize/tolerances_and_stopping/
    if ((dx->cwiseAbs().array() < (rel_tol_ * qp_.x_scale).array()).all()) {
      return x_;
    }
  }
}

size_t SQP::iterations() const
{
  return iter_;
}

void SQP::setMaximumIterations(size_t _max_iter)
{
  max_iter_ = _max_iter;
}

void SQP::setRelativeTolerance(double _rel_tol)
{
  assert(_rel_tol > 0.0);
  rel_tol_ = _rel_tol;
}

void SQP::setVariableScales(const Eigen::VectorXd& _x_scale)
{
  assert(_x_scale.size() == n_);
  assert((_x_scale.array() > 0.0).all());
  qp_.x_scale = _x_scale;
}
}  // namespace nlp
}  // namespace tobas
