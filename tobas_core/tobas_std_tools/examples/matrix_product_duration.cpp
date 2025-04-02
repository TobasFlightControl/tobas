#include <iostream>
#include <eigen3/Eigen/Core>

#include <tobas_std_tools/stopwatch.hpp>

using namespace std;
using namespace std::chrono;
using namespace Eigen;

template <typename T>
void process(int n, int iter)
{
  const auto A = Matrix<T, Dynamic, Dynamic>::Random(n, n);
  const auto B = Matrix<T, Dynamic, Dynamic>::Random(n, n);

  tobas_std::Stopwatch stopwatch(iter);

  for (int i = 0; i < iter; ++i)
  {
    cout << "Iter " << i << endl;

    stopwatch.start();
    const auto C = (A * B).eval();
    stopwatch.stop();
  }
}

int main(int argc, char* argv[])
{
  if (argc < 4)
  {
    cerr << "Usage: " << argv[0] << " <Scalar> <Matrix Size> <Iterations>" << endl;
    return EXIT_FAILURE;
  }

  const auto scalar = argv[1];
  const auto n = atoi(argv[2]);
  const auto iter = atoi(argv[3]);

  if (strcmp(scalar, "char") == 0)
    process<char>(n, iter);
  else if (strcmp(scalar, "short") == 0)
    process<short>(n, iter);
  else if (strcmp(scalar, "int") == 0)
    process<int>(n, iter);
  else if (strcmp(scalar, "long") == 0)
    process<long>(n, iter);
  else if (strcmp(scalar, "float") == 0)
    process<float>(n, iter);
  else if (strcmp(scalar, "double") == 0)
    process<double>(n, iter);
  else
  {
    cerr << "Invalid scalar type: " << scalar << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
