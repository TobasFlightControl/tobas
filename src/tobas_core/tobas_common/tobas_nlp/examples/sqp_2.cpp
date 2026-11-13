// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

/**
 * min  x0 * x1 * (x0 + x1 + x2) + x2
 * s.t. x0^2 + x1^2 + x2^2 + x3^2 = 04
 *      x0 * x1 * x2 * x3 >= 25
 *      1 <= x0, x1, x2, x3 <= 5
 */

#include <iostream>

#include <tobas_math/core.hpp>
#include <tobas_nlp/sqp.hpp>

using namespace std;
using namespace Eigen;

double f(const VectorXd& _x)
{
  return _x(0) * _x(3) * (_x(0) + _x(1) + _x(2)) + _x(2);
}

VectorXd g(const VectorXd& _x)
{
  VectorXd res(9);
  res(0) = 25 - _x(0) * _x(1) * _x(2) * _x(3);
  res(1) = -_x(0) - 1;
  res(2) = -_x(1) - 1;
  res(3) = -_x(2) - 1;
  res(4) = -_x(3) - 1;
  res(5) = _x(0) - 5;
  res(6) = _x(1) - 5;
  res(7) = _x(2) - 5;
  res(8) = _x(3) - 5;
  return res;
}

VectorXd h(const VectorXd& _x)
{
  VectorXd res(1);
  res(0) = tobas::math::sqr(_x(0)) + tobas::math::sqr(_x(1)) + tobas::math::sqr(_x(2)) + tobas::math::sqr(_x(3)) - 40;
  return res;
}

RowVectorXd dfdx(const VectorXd& _x)
{
  RowVectorXd res(4);
  res(0) = _x(3) * (2 * _x(0) + _x(1) + _x(2));
  res(1) = _x(0) * _x(3);
  res(2) = _x(0) * _x(3) + 1;
  res(3) = _x(0) * (_x(0) + _x(1) + _x(2));
  return res;
}

MatrixXd dgdx(const VectorXd& _x)
{
  MatrixXd res(9, 4);

  res(0, 0) = -_x(1) * _x(2) * _x(3);
  res(0, 1) = -_x(0) * _x(2) * _x(3);
  res(0, 2) = -_x(0) * _x(1) * _x(3);
  res(0, 3) = -_x(0) * _x(1) * _x(2);

  res.block(1, 0, 4, 4).setZero();
  res.block(1, 0, 4, 4).diagonal().fill(-1);
  res.block(5, 0, 4, 4).setZero();
  res.block(5, 0, 4, 4).diagonal().fill(1);

  return res;
}

MatrixXd dhdx(const VectorXd& _x)
{
  MatrixXd res(1, 4);
  res(0, 0) = 2 * _x(0);
  res(0, 1) = 2 * _x(1);
  res(0, 2) = 2 * _x(2);
  res(0, 3) = 2 * _x(3);
  return res;
}

MatrixXd dFdx(const VectorXd& _x)
{
  MatrixXd res(4, 4);
  res(0, 0) = 2 * _x(3);
  res(0, 1) = _x(3);
  res(0, 2) = _x(3);
  res(0, 3) = 2 * _x(0) + _x(1) + _x(2);
  res(1, 0) = _x(3);
  res(1, 1) = 0;
  res(1, 2) = 0;
  res(1, 3) = _x(0);
  res(2, 0) = _x(3);
  res(2, 1) = 0;
  res(2, 2) = 0;
  res(2, 3) = _x(0);
  res(3, 0) = 2 * _x(0) + _x(1) + _x(2);
  res(3, 1) = _x(0);
  res(3, 2) = _x(0);
  res(3, 3) = 0;
  return res;
}

Tensor3Xd dGdx(const VectorXd& _x)
{
  Tensor3Xd res(9, 4, 4);
  res.setZero();

  res(0, 0, 0) = 0;
  res(0, 0, 1) = -_x(2) * _x(3);
  res(0, 0, 2) = -_x(1) * _x(3);
  res(0, 0, 3) = -_x(1) * _x(2);

  res(0, 1, 0) = -_x(2) * _x(3);
  res(0, 1, 1) = 0;
  res(0, 1, 2) = -_x(0) * _x(3);
  res(0, 1, 3) = -_x(0) * _x(2);

  res(0, 2, 0) = -_x(1) * _x(3);
  res(0, 2, 1) = -_x(0) * _x(3);
  res(0, 2, 2) = 0;
  res(0, 2, 3) = -_x(0) * _x(1);

  res(0, 3, 0) = -_x(1) * _x(2);
  res(0, 3, 1) = -_x(0) * _x(2);
  res(0, 3, 2) = -_x(0) * _x(1);
  res(0, 3, 3) = 0;

  return res;
}

Tensor3Xd dHdx(const VectorXd&)
{
  Tensor3Xd res(1, 4, 4);
  res.setZero();

  res(0, 0, 0) = 2;
  res(0, 1, 1) = 2;
  res(0, 2, 2) = 2;
  res(0, 3, 3) = 2;

  return res;
}

int main()
{
  tobas::nlp::SQP sqp;

  VectorXd x0(4);
  x0 << 1, 5, 5, 1;

  sqp.initialize(x0, f, g, h, dfdx, dgdx, dhdx, dFdx, dGdx, dHdx);

  const auto x_opt = sqp.solve();
  if (!x_opt) {
    cerr << x_opt.error() << '\n';
    return EXIT_FAILURE;
  }

  cout << "Optimal solution: " << x_opt->transpose() << '\n';
  cout << "Number of iterations: " << sqp.iterations() << '\n';  // 5 with scipy.optimize.minimize.
  cout << "----------\n";
  cout << "f(x*) = " << f(*x_opt) << '\n';  // 17.014017247073575 with scipy.optimize.minimize.
  cout << "g(x*) = " << g(*x_opt).transpose() << '\n';
  cout << "h(x*) = " << h(*x_opt).transpose() << '\n';

  return EXIT_SUCCESS;
}
