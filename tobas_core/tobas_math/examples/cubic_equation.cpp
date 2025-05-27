#include <iostream>

#include <tobas_math/complex.hpp>
#include <tobas_math/equation.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 5) {
    cerr << "Usage: " << argv[0] << " a b c d -> Solve: a x^3 + b x^2 + c x + d = 0" << endl;
    return EXIT_FAILURE;
  }
  const auto a = stod(argv[1]);
  const auto b = stod(argv[2]);
  const auto c = stod(argv[3]);
  const auto d = stod(argv[4]);

  const auto [x1, x2, x3] = math::solveCubicEquation(a, b, c, d);
  cout << a << " x^3 + " << b << " x^2 + " << c << " x + " << d << " = 0 <=> x = " << x1 << ", " << x2 << ", " << x3
       << endl;
}
