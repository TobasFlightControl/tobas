#pragma once

#include <complex>
#include <tuple>

namespace math
{
/* Solve a x + b = 0 (a != 0) */
double solveLinearEquation(double a, double b);

/* Solve a x^2 + b x + c = 0 (a != 0) */
std::pair<std::complex<double>, std::complex<double>> solveQuadraticEquation(double a, double b, double c);

/**
 * Solve a x^3 + b x^2 + c x + d = 0 (a != 0)
 *
 * cf. https://oshima-gakushujuku.com/blog/math/formula-qubic-equation/
 */
std::tuple<std::complex<double>, std::complex<double>, std::complex<double>>
solveCubicEquation(double a, double b, double c, double d);
}  // namespace math
