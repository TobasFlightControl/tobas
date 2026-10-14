// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/util.hpp"

#include <tobas_eigen_tools/linalg.hpp>
#include <tobas_quadprog/utils.hpp>

namespace tobas::ctrl
{
namespace
{
constexpr auto kEps = std::numeric_limits<double>::epsilon();
}  // namespace

Eigen::MatrixXd ctrb(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _b)
{
  const auto x_size = _a.rows();
  const auto u_size = _b.cols();

  assert(_a.rows() == x_size && _a.cols() == x_size);
  assert(_b.rows() == x_size && _b.cols() == u_size);

  Eigen::MatrixXd Mc(x_size, u_size * x_size);
  Eigen::MatrixXd tmp = _b;
  for (int i = 0; i < x_size; ++i) {
    Mc.block(0, u_size * i, x_size, u_size) = tmp;
    tmp = _a * tmp;
  }

  return Mc;
}

Eigen::MatrixXd obsv(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _c)
{
  const auto x_size = _a.rows();
  const auto y_size = _c.rows();

  assert(_a.rows() == x_size && _a.cols() == x_size);
  assert(_c.rows() == y_size && _c.cols() == x_size);

  Eigen::MatrixXd Mo(y_size * x_size, x_size);
  Eigen::MatrixXd tmp = _c;
  for (int i = 0; i < x_size; ++i) {
    Mo.block(y_size * i, 0, y_size, x_size) = tmp;
    tmp = tmp * _a;
  }

  return Mo;
}

bool isControllable(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _b)
{
  const auto x_size = _a.rows();
  const auto Mc = ctrb(_a, _b);
  const auto rank = eigen::matrixRank(Mc);
  return rank == x_size;
}

bool isObservable(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _c)
{
  const auto x_size = _a.rows();
  const auto Mo = obsv(_a, _c);
  const auto rank = eigen::matrixRank(Mo);
  return rank == x_size;
}

LinearEquation matIneqFromRange(const Eigen::VectorXd& _lb, const Eigen::VectorXd& _ub, double _inf)
{
  LinearEquation res;
  quadprog::matIneqFromRange(_lb, _ub, res.A, res.b, _inf);
  return res;
}

double firstOrderPos(double _x0, double _xd, double _tau, double _t)
{
  return _xd - std::exp(-_t / (_tau + kEps)) * (_xd - _x0);
}

double firstOrderVel(double _x0, double _v0, double _vd, double _tau, double _t)
{
  return _x0 + _vd * _t + _tau * std::expm1(-_t / (_tau + kEps)) * (_vd - _v0);
}
}  // namespace tobas::ctrl
