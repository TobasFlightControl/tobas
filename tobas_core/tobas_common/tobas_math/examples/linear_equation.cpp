// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_math/equation.hpp>

using namespace std;

int main(int _argc, char** _argv)
{
  if (_argc != 3) {
    cerr << "Usage: " << _argv[0] << " a b -> Solve: ax + b = 0" << endl;
    return EXIT_FAILURE;
  }
  const auto a = stod(_argv[1]);
  const auto b = stod(_argv[2]);

  const auto x = tobas::math::solveLinearEquation(a, b);
  cout << a << " x + " << b << " = 0 <=> x = " << x << endl;
}
