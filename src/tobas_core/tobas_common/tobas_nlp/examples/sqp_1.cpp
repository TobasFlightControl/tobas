// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

/** min  x0^2 + x0 * x1 s.t. x0^3 + x0 * x1 = 100 x0^2 + 4 * x1 >= 50 -100 <= x0, x1 <= 100 */

#include <cstdlib>
#include <iostream>

#include <eigen3/Eigen/Core>

#include <tobas_eigen_tools/tensor.hpp>
#include <tobas_math/core.hpp>
#include <tobas_nlp/sqp.hpp>

double f(const Eigen::VectorXd& _x)
{
  return tobas::math::sqr(_x(0)) + _x(0) * _x(1);
}

Eigen::VectorXd g(const Eigen::VectorXd& _x)
{
  Eigen::VectorXd res(5);
  res(0) = 50 - tobas::math::sqr(_x(0)) - 4 * _x(1);
  res(1) = -_x(0) - 100;
  res(2) = -_x(1) - 100;
  res(3) = _x(0) - 100;
  res(4) = _x(1) - 100;
  return res;
}

Eigen::VectorXd h(const Eigen::VectorXd& _x)
{
  Eigen::VectorXd res(1);
  res(0) = tobas::math::cube(_x(0)) + _x(0) * _x(1) - 100;
  return res;
}

Eigen::RowVectorXd dfdx(const Eigen::VectorXd& _x)
{
  Eigen::RowVectorXd res(2);
  res(0) = 2 * _x(0) + _x(1);
  res(1) = _x(0);
  return res;
}

Eigen::MatrixXd dgdx(const Eigen::VectorXd& _x)
{
  Eigen::MatrixXd res(5, 2);
  res(0, 0) = -2 * _x(0);
  res(0, 1) = -4;
  res(1, 0) = -1;
  res(1, 1) = 0;
  res(2, 0) = 0;
  res(2, 1) = -1;
  res(3, 0) = 1;
  res(3, 1) = 0;
  res(4, 0) = 0;
  res(4, 1) = 1;
  return res;
}

Eigen::MatrixXd dhdx(const Eigen::VectorXd& _x)
{
  Eigen::MatrixXd res(1, 2);
  res(0, 0) = 3 * tobas::math::sqr(_x(0)) + _x(1);
  res(0, 1) = _x(0);
  return res;
}

Eigen::MatrixXd dFdx(const Eigen::VectorXd&)
{
  Eigen::MatrixXd res(2, 2);
  res(0, 0) = 2;
  res(0, 1) = 1;
  res(1, 0) = 1;
  res(1, 1) = 0;
  return res;
}

Eigen::Tensor3Xd dGdx(const Eigen::VectorXd&)
{
  Eigen::Tensor3Xd res(5, 2, 2);
  res.setZero();
  res(0, 0, 0) = -2;
  return res;
}

Eigen::Tensor3Xd dHdx(const Eigen::VectorXd& _x)
{
  Eigen::Tensor3Xd res(1, 2, 2);
  res(0, 0, 0) = 6 * _x(0);
  res(0, 0, 1) = 1;
  res(0, 1, 0) = 1;
  res(0, 1, 1) = 0;
  return res;
}

int main()
{
  tobas::nlp::SQP sqp;

  Eigen::VectorXd x0(2);
  x0 << 5, 5;

  sqp.initialize(x0, f, g, h, dfdx, dgdx, dhdx, dFdx, dGdx, dHdx);

  const auto x_opt = sqp.solve();
  if (!x_opt) {
    std::cerr << x_opt.error() << '\n';
    return EXIT_FAILURE;
  }

  std::cout << "Optimal solution: " << x_opt->transpose() << '\n';  // 4.04092525, 8.41773078
  std::cout << "Number of iterations: " << sqp.iterations() << '\n';
  std::cout << "----------\n";
  std::cout << "f(x*) = " << f(*x_opt) << '\n';
  std::cout << "g(x*) = " << g(*x_opt).transpose() << '\n';
  std::cout << "h(x*) = " << h(*x_opt).transpose() << '\n';

  return EXIT_SUCCESS;
}
