#include <iostream>

#include <tobas_math/equation.hpp>
#include <tobas_math/complex.hpp>

using namespace std;

int main()
{
  constexpr double a = 1;
  constexpr double b = 1;
  constexpr double c = 1;

  const auto [x1, x2] = math::solveQuadraticEquation(a, b, c);
  cout << a << " x^2 + " << b << " x + " << c << " = 0 <=> x = " << x1 << ", " << x2 << endl;
}
