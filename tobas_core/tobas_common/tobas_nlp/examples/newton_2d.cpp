// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

/**
 * Solve: x^2 + 2x - 1 = 0
 */

#include <iostream>

#include <tobas_math/core.hpp>
#include <tobas_nlp/newton_2d.hpp>

using namespace std;

Eigen::Vector2d f(const Eigen::Vector2d& x_vec)
{
  const auto x = x_vec(0);
  const auto y = x_vec(1);
  return Eigen::Vector2d(
    tobas::math::sqr(x) + tobas::math::sqr(y) - 4,
    tobas::math::sqr(x) - y - 1
  );
}

Eigen::Matrix2d dfdx(const Eigen::Vector2d& x_vec)
{
  const auto x = x_vec(0);
  const auto y = x_vec(1);
  Eigen::Matrix2d J;
  J << 2.0 * x, 2.0 * y,
      2.0 * x, -1.0;

  return J;
}

int main()
{
  tobas::nlp::NewtonSolver2d newton;

  newton.initialize(f, dfdx);

  Eigen::Vector2d x(1.5, 1.0);
  if (newton.solve(x) < 0) {
    cerr << newton.errorMessage() << endl;
    return EXIT_FAILURE;
  }

  cout << "Optimal solution: " << x.transpose() << endl;  // 1.51749, 1.30278
  cout << "----------" << endl;
  cout << "f(x*) = " << f(x).transpose() << endl;

  return EXIT_SUCCESS;
}
