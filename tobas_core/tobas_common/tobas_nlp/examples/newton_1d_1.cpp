// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

/** Solve: x^2 + 2x - 1 = 0 */

#include <iostream>

#include <tobas_math/core.hpp>
#include <tobas_nlp/newton_1d.hpp>

using namespace std;

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
    cerr << x.error() << endl;
    return EXIT_FAILURE;
  }

  cout << "Optimal solution: " << *x << endl;  // 0.4142
  cout << "----------" << endl;
  cout << "f(x*) = " << f(*x) << endl;

  return EXIT_SUCCESS;
}
