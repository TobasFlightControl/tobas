// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/util.hpp"

#include <tobas_eigen_tools/linalg.hpp>
#include <tobas_quadprog/utils.hpp>

namespace tobas
{
namespace ctrl
{
namespace
{
constexpr auto kEps = std::numeric_limits<double>::epsilon();
}  // namespace

Eigen::MatrixXd ctrb(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{
  const auto x_size = A.rows();
  const auto u_size = B.cols();

  assert(A.rows() == x_size && A.cols() == x_size);
  assert(B.rows() == x_size && B.cols() == u_size);

  Eigen::MatrixXd Mc(x_size, u_size * x_size);
  Eigen::MatrixXd tmp = B;
  for (int i = 0; i < x_size; ++i) {
    Mc.block(0, u_size * i, x_size, u_size) = tmp;
    tmp = A * tmp;
  }

  return Mc;
}

Eigen::MatrixXd obsv(const Eigen::MatrixXd& A, const Eigen::MatrixXd& C)
{
  const auto x_size = A.rows();
  const auto y_size = C.rows();

  assert(A.rows() == x_size && A.cols() == x_size);
  assert(C.rows() == y_size && C.cols() == x_size);

  Eigen::MatrixXd Mo(y_size * x_size, x_size);
  Eigen::MatrixXd tmp = C;
  for (int i = 0; i < x_size; ++i) {
    Mo.block(y_size * i, 0, y_size, x_size) = tmp;
    tmp = tmp * A;
  }

  return Mo;
}

bool isControllable(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{
  const auto x_size = A.rows();
  const auto Mc = ctrb(A, B);
  const auto rank = eigen::matrixRank(Mc);
  return rank == x_size;
}

bool isObservable(const Eigen::MatrixXd& A, const Eigen::MatrixXd& C)
{
  const auto x_size = A.rows();
  const auto Mo = obsv(A, C);
  const auto rank = eigen::matrixRank(Mo);
  return rank == x_size;
}

LinearEquation matIneqFromRange(const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, double inf)
{
  LinearEquation res;
  quadprog::matIneqFromRange(lb, ub, res.A, res.b, inf);
  return res;
}

double firstOrderPos(double x0, double xd, double tau, double t)
{
  return xd - std::exp(-t / (tau + kEps)) * (xd - x0);
}

double firstOrderVel(double x0, double v0, double vd, double tau, double t)
{
  return x0 + vd * t - tau * (1 - std::exp(-t / (tau + kEps))) * (vd - v0);
}
}  // namespace ctrl
}  // namespace tobas
