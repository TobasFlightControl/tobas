#include <iostream>

#include <tobas_math/equation.hpp>

using namespace std;

int main()
{
  constexpr double a = 2;
  constexpr double b = 3;

  const auto x = math::solveLinearEquation(a, b);
  cout << a << " x + " << b << " = 0 <=> x = " << x << endl;
}
