/**
 * Solve: x^2 + 2x - 1 = 0
 */

#include <iostream>

#include <tobas_math/core.hpp>
#include <tobas_nlp/newton_1d.hpp>

using namespace std;

double f(double x)
{
  return math::sqr(x) + 2 * x - 1;
}

double dfdx(double x)
{
  return 2 * x + 2;
}

int main()
{
  nlp::NewtonSolver1d newton;

  newton.initialize(f, dfdx);

  double x = 1;
  if (newton.solve(x) < 0) {
    cerr << newton.errorMessage() << endl;
    return EXIT_FAILURE;
  }

  cout << "Optimal solution: " << x << endl;  // 0.4142
  cout << "----------" << endl;
  cout << "f(x*) = " << f(x) << endl;

  return EXIT_SUCCESS;
}
