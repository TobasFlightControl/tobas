#include <iostream>

#include <tobas_math/equation.hpp>
#include <tobas_math/complex.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 4)
  {
    cerr << "Usage: " << argv[0] << " a b c -> Solve: a x^2 + b x + c = 0" << endl;
    return EXIT_FAILURE;
  }
  const auto a = stod(argv[1]);
  const auto b = stod(argv[2]);
  const auto c = stod(argv[3]);

  const auto [x1, x2] = math::solveQuadraticEquation(a, b, c);
  cout << a << " x^2 + " << b << " x + " << c << " = 0 <=> x = " << x1 << ", " << x2 << endl;
}
