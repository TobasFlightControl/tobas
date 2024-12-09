#include <random>

#include <tobas_std_tools/vector.hpp>

#include <tobas_dsp/welford.hpp>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    cerr << "Usage: " << argv[0] << " <Data Length>" << endl;
    return EXIT_FAILURE;
  }

  const auto length = stoul(argv[1]);

  // Define random generator
  random_device rnd_dev;
  mt19937 rnd_gen(rnd_dev());
  uniform_real_distribution<double> uniform(0., 1.);

  // Create data
  vector<Vector3d> data;
  for (size_t _ = 0; _ < length; ++_)
    data.emplace_back(uniform(rnd_gen), uniform(rnd_gen), uniform(rnd_gen));

  // Compute variance with normal method
  Vector3d data_sum = Vector3d::Zero();
  for (const auto& x : data)
    data_sum += x;
  const Vector3d mean_1 = data_sum / length;

  Matrix3d cov_sum = Matrix3d::Zero();
  for (const auto& x : data)
  {
    const Vector3d d = x - mean_1;
    cov_sum += d * d.transpose();
  }
  const Matrix3d cov_1 = cov_sum / length;

  // Compute variance with Welford method
  dsp::Welford<double, 3> welford;
  for (const auto& x : data)
    welford.add(x);
  const Vector3d& mean_2 = welford.mean();
  const Matrix3d& cov_2 = welford.variance();

  // Show results
  cout << "Mean (Normal Method) : " << mean_1.transpose() << endl;
  cout << "Mean (Welford Method): " << mean_2.transpose() << endl;
  cout << "Coariance (Normal Method):" << endl << cov_1 << endl;
  cout << "Coariance (Welford Method):" << endl << cov_2 << endl;

  // Validate
  if (!eigen::isClose(mean_1, mean_2) || !eigen::isClose(cov_1, cov_2))
  {
    cerr << "Welford method is inaccurate." << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
