// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstdlib>
#include <iostream>
#include <string>

#include <tobas_math/equation.hpp>

int main(int _argc, char* _argv[])
{
  if (_argc != 3) {
    std::cerr << "Usage: " << _argv[0] << " a b -> Solve: ax + b = 0\n";
    return EXIT_FAILURE;
  }
  const auto a = std::stod(_argv[1]);
  const auto b = std::stod(_argv[2]);

  const auto x = tobas::math::solveLinearEquation(a, b);
  std::cout << a << " x + " << b << " = 0 <=> x = " << x << '\n';
}
