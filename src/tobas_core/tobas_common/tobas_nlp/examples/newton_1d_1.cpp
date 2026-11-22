// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

/** Solve: x^2 + 2x - 1 = 0 */

#include <cstdlib>
#include <iostream>

#include <tobas_math/core.hpp>
#include <tobas_nlp/newton_1d.hpp>

double f(double _x)
{
  return tobas::math::sqr(_x) + 2 * _x - 1;
}

double dfdx(double _x)
{
  return 2 * _x + 2;
}

int main()
{
  tobas::nlp::NewtonSolver1d newton;

  newton.initialize(f, dfdx);

  const auto x = newton.solve(1.0);
  if (!x) {
    std::cerr << x.error() << '\n';
    return EXIT_FAILURE;
  }

  std::cout << "Optimal solution: " << *x << '\n';  // 0.4142
  std::cout << "----------\n";
  std::cout << "f(x*) = " << f(*x) << '\n';

  return EXIT_SUCCESS;
}
