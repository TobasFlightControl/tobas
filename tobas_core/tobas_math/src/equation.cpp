#include <cassert>
#include <iostream>

#include "../include/tobas_math/equation.hpp"
#include "../include/tobas_math/core.hpp"
#include "../include/tobas_math/complex.hpp"

using namespace std;

namespace math
{
double solveLinearEquation(double a, double b)
{
  assert(a != 0);
  return -b / a;
}

pair<complex<double>, complex<double>> solveQuadraticEquation(double a, double b, double c)
{
  assert(a != 0);

  const complex<double> d = sqr(b) - 4 * a * c;
  const auto sqrt_d = sqrt(d);
  const auto a2 = 2 * a;

  return { (-b + sqrt_d) / a2, (-b - sqrt_d) / a2 };
}

tuple<complex<double>, complex<double>, complex<double>> solveCubicEquation(double a, double b, double c, double d)
{
  assert(a != 0);

  const auto real = -b / (3 * a);

  const auto a2 = a * a;
  const auto a3 = a2 * a;
  const auto b2 = b * b;
  const auto b3 = b2 * b;

  const auto p = (-b2 + 3 * a * c) / (9 * a2);
  const auto q = (2 * b3 - 9 * a * b * c + 27 * a2 * d) / (54 * a3);

  const complex<double> q2p3 = sqr(q) + cube(p);
  const auto sqrt_q2p3 = sqrt(q2p3);
  const auto cbrt_plus = cbrt(-q + sqrt_q2p3);
  const auto cbrt_minus = cbrt(-q - sqrt_q2p3);

  constexpr complex<double> omega(-0.5, +numbers::sqrt3 / 2);
  constexpr complex<double> omega2(-0.5, -numbers::sqrt3 / 2);

  const auto x1 = real + cbrt_plus + cbrt_minus;
  const auto x2 = real + omega * cbrt_plus + omega2 * cbrt_minus;
  const auto x3 = real + omega2 * cbrt_plus + omega * cbrt_minus;
  return { x1, x2, x3 };
}
}  // namespace math
