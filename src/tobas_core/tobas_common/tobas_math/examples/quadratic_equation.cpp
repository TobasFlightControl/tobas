// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstdlib>
#include <iostream>
#include <string>

#include <tobas_math/complex.hpp>
#include <tobas_math/equation.hpp>

int main(int _argc, char* _argv[])
{
  if (_argc != 4) {
    std::cerr << "Usage: " << _argv[0] << " a b c -> Solve: a x^2 + b x + c = 0\n";
    return EXIT_FAILURE;
  }
  const auto a = std::stod(_argv[1]);
  const auto b = std::stod(_argv[2]);
  const auto c = std::stod(_argv[3]);

  const auto [x1, x2] = tobas::math::solveQuadraticEquation(a, b, c);
  std::cout << a << " x^2 + " << b << " x + " << c << " = 0 <=> x = " << x1 << ", " << x2 << '\n';
}
