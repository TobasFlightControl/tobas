#include <iostream>

#include <tobas_math/equation.hpp>
#include <tobas_math/complex.hpp>

using namespace std;

int main()
{
  constexpr double a = 1;
  constexpr double b = 2;
  constexpr double c = 2;
  constexpr double d = 1;

  const auto [x1, x2, x3] = math::solveCubicEquation(a, b, c, d);
  cout << a << " x^3 + " << b << " x^2 + " << c << " x + " << d << " = 0 <=> x = " << x1 << ", " << x2 << ", " << x3
       << endl;
}
