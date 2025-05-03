#include <random>

#include <tobas_std_tools/vector.hpp>

#include <tobas_dsp/moving_stat.hpp>

#define LENGTH 1000

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  if (argc != 2) {
    cerr << "Usage: " << argv[0] << " <Steps>" << endl;
    return EXIT_FAILURE;
  }

  const auto steps = stoul(argv[1]);

  // Define random generator
  random_device rnd_dev;
  mt19937 rnd_gen(rnd_dev());
  uniform_real_distribution<double> uniform(0., 1.);

  // Create data
  array<Vector3d, LENGTH> init_data;
  vector<Vector3d> incoming_data(steps);
  vector<Vector3d> total_data(LENGTH + steps);
  for (size_t i = 0; i < LENGTH; ++i) {
    Vector3d data(uniform(rnd_gen), uniform(rnd_gen), uniform(rnd_gen));
    init_data.at(i) = data;
    total_data.at(i) = data;
  }
  for (size_t i = 0; i < steps; ++i) {
    Vector3d data(uniform(rnd_gen), uniform(rnd_gen), uniform(rnd_gen));
    incoming_data.at(i) = data;
    total_data.at(LENGTH + i) = data;
  }

  // Initialize MovingStatistics object
  dsp::MovingStatistics<double, 3, LENGTH> moving_stat;
  moving_stat.initialize(init_data);

  for (size_t i = 0; i < steps; ++i) {
    // Compute moving statistics with offline method
    Vector3d data_sum = Vector3d::Zero();
    for (size_t j = i; j < i + LENGTH; ++j) {
      data_sum += total_data.at(j);
    }
    const Vector3d mean_1 = data_sum / LENGTH;

    Matrix3d cov_sum = Matrix3d::Zero();
    for (size_t j = i; j < i + LENGTH; ++j) {
      const Vector3d d = total_data.at(j) - mean_1;
      cov_sum += d * d.transpose();
    }
    const Matrix3d cov_1 = cov_sum / LENGTH;

    // Compute moving statistics with online method
    const auto mean_2 = moving_stat.mean();
    const auto cov_2 = moving_stat.variance();

    // Show results
    cout << "Start Index: " << i << ", Window Size: " << LENGTH << endl;
    cout << "Mean (Normal Method): " << mean_1.transpose() << endl;
    cout << "Mean (Moving Stat)  : " << mean_2.transpose() << endl;
    cout << "Covariance (Normal Method):" << endl << cov_1 << endl;
    cout << "Covariance (Moving Stat)  :" << endl << cov_2 << endl;
    cout << "----------" << endl;

    // Validate
    if (!eigen::isClose(mean_1, mean_2) || !eigen::isClose(cov_1, cov_2)) {
      cerr << "Moving statistics is inaccurate." << endl;
      return EXIT_FAILURE;
    }

    // Add next data
    moving_stat.add(incoming_data[i]);
  }

  return EXIT_SUCCESS;
}
