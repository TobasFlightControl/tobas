// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_math/complex.hpp>
#include <tobas_math/equation.hpp>

using namespace std;

int main(int _argc, char** _argv)
{
  if (_argc != 5) {
    cerr << "Usage: " << _argv[0] << " a b c d -> Solve: a x^3 + b x^2 + c x + d = 0" << endl;
    return EXIT_FAILURE;
  }
  const auto a = stod(_argv[1]);
  const auto b = stod(_argv[2]);
  const auto c = stod(_argv[3]);
  const auto d = stod(_argv[4]);

  const auto [x1, x2, x3] = tobas::math::solveCubicEquation(a, b, c, d);
  cout << a << " x^3 + " << b << " x^2 + " << c << " x + " << d << " = 0 <=> x = " << x1 << ", " << x2 << ", " << x3
       << endl;
}
