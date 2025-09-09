#include <iostream>

#include <tobas_math/equation.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 3) {
    cerr << "Usage: " << argv[0] << " a b -> Solve: ax + b = 0" << endl;
    return EXIT_FAILURE;
  }
  const auto a = stod(argv[1]);
  const auto b = stod(argv[2]);

  const auto x = math::solveLinearEquation(a, b);
  cout << a << " x + " << b << " = 0 <=> x = " << x << endl;
}
