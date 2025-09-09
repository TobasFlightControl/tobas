/**
 * Solve: x^3 + 2 x^2 + 10 x - 20 = 0
 */

#include <iostream>

#include <tobas_math/core.hpp>
#include <tobas_nlp/newton_1d.hpp>

using namespace std;

double f(double x)
{
  return math::cube(x) + 2 * math::sqr(x) + 10 * x - 20;
}

double dfdx(double x)
{
  return 3 * math::sqr(x) + 4 * x + 10;
}

int main()
{
  nlp::NewtonSolver1d newton;

  newton.initialize(f, dfdx);

  double x = 1.5;
  if (newton.solve(x) < 0) {
    cerr << newton.errorMessage() << endl;
    return EXIT_FAILURE;
  }

  cout << "Optimal solution: " << x << endl;  // 1.3688081
  cout << "----------" << endl;
  cout << "f(x*) = " << f(x) << endl;

  return EXIT_SUCCESS;
}
