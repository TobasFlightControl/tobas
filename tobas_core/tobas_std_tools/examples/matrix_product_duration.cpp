#include <iostream>
#include <eigen3/Eigen/Core>

#include <tobas_std_tools/stopwatch.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    cerr << "Usage: " << argv[0] << " <Size> <Trials>" << endl;
    return EXIT_FAILURE;
  }

  const auto size = stoul(argv[1]);
  const auto trials = stoul(argv[2]);

  const auto A = Eigen::MatrixXf::Random(size, size);
  const auto B = Eigen::MatrixXf::Random(size, size);

  tobas_std::Stopwatch stopwatch(trials);

  for (size_t _ = 0; _ < trials; ++_)
  {
    stopwatch.start();
    const auto C = (A * B).eval();
    stopwatch.stop();
  }

  return EXIT_SUCCESS;
}
