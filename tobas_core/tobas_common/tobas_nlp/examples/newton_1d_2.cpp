// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

/** Solve: x^3 + 2 x^2 + 10 x - 20 = 0 */

#include <iostream>

#include <tobas_math/core.hpp>
#include <tobas_nlp/newton_1d.hpp>

using namespace std;

double f(double _x)
{
  return tobas::math::cube(_x) + 2 * tobas::math::sqr(_x) + 10 * _x - 20;
}

double dfdx(double _x)
{
  return 3 * tobas::math::sqr(_x) + 4 * _x + 10;
}

int main()
{
  tobas::nlp::NewtonSolver1d newton;

  newton.initialize(f, dfdx);

  const auto x = newton.solve(1.5);
  if (!x) {
    cerr << x.error() << '\n';
    return EXIT_FAILURE;
  }

  cout << "Optimal solution: " << *x << '\n';  // 1.3688081
  cout << "----------\n";
  cout << "f(x*) = " << f(*x) << '\n';

  return EXIT_SUCCESS;
}
