// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_math/equation.hpp"

#include <cassert>

#include "tobas_math/complex.hpp"
#include "tobas_math/core.hpp"

namespace tobas::math
{
double solveLinearEquation(double _a, double _b)
{
  assert(_a != 0);
  return -_b / _a;
}

std::pair<std::complex<double>, std::complex<double>> solveQuadraticEquation(double _a, double _b, double _c)
{
  assert(_a != 0);

  const std::complex<double> d = sqr(_b) - 4 * _a * _c;
  const auto sqrt_d = sqrt(d);
  const auto a2 = 2 * _a;

  return { (-_b + sqrt_d) / a2, (-_b - sqrt_d) / a2 };
}

std::tuple<std::complex<double>, std::complex<double>, std::complex<double>>
solveCubicEquation(double _a, double _b, double _c, double _d)
{
  assert(_a != 0);

  const auto real = -_b / (3 * _a);

  const auto a2 = _a * _a;
  const auto a3 = a2 * _a;
  const auto b2 = _b * _b;
  const auto b3 = b2 * _b;

  const auto p = (-b2 + 3 * _a * _c) / (9 * a2);
  const auto q = (2 * b3 - 9 * _a * _b * _c + 27 * a2 * _d) / (54 * a3);

  const std::complex<double> q2p3 = sqr(q) + cube(p);
  const auto sqrt_q2p3 = sqrt(q2p3);
  const auto cbrt_plus = cbrt(-q + sqrt_q2p3);
  const auto cbrt_minus = cbrt(-q - sqrt_q2p3);

  constexpr std::complex<double> omega(-0.5, +std::numbers::sqrt3 / 2);
  constexpr std::complex<double> omega2(-0.5, -std::numbers::sqrt3 / 2);

  const auto x1 = real + cbrt_plus + cbrt_minus;
  const auto x2 = real + omega * cbrt_plus + omega2 * cbrt_minus;
  const auto x3 = real + omega2 * cbrt_plus + omega * cbrt_minus;
  return { x1, x2, x3 };
}
}  // namespace tobas::math
