#include <random>

#include <tobas_std_tools/vector.hpp>
#include <tobas_std_tools/float.hpp>

#include <tobas_dsp/moving_stat.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    cerr << "Usage: " << argv[0] << " <Window Size> <Steps>" << endl;
    return EXIT_FAILURE;
  }

  const auto window_size = atoi(argv[1]);
  const auto steps = atoi(argv[2]);

  // Define random generator
  random_device rnd_dev;
  mt19937 rnd_gen(rnd_dev());
  uniform_real_distribution<double> uniform(0., 1.);

  // Create data
  vector<double> init_data, incoming_data;
  for (int _ = 0; _ < window_size; ++_)
    init_data.push_back(uniform(rnd_gen));
  for (int _ = 0; _ < steps; ++_)
    incoming_data.push_back(uniform(rnd_gen));
  const auto data = tobas_std::merge(init_data, incoming_data);
  cout << "Data: " << data << endl;

  // Initialize MovingStatistics object
  dsp::MovingStatistics moving_stat_;
  moving_stat_.initialize(init_data);

  for (int i = 0; i < steps; ++i)
  {
    // Compute moving statistics in 2 methods
    const auto mean_1 = tobas_std::fmean(data, i, window_size);
    const auto var_1 = tobas_std::variance(data, i, window_size);
    const auto mean_2 = moving_stat_.mean();
    const auto var_2 = moving_stat_.variance();

    // Show results
    cout << "Start Index: " << i << ", Window Size: " << window_size << endl;
    cout << "\tMean (Normal Method)    : " << mean_1 << endl;
    cout << "\tMean (Moving Stat)      : " << mean_2 << endl;
    cout << "\tVariance (Normal Method): " << var_1 << endl;
    cout << "\tVariance (Moving Stat)  : " << var_2 << endl;

    // Validate
    if (!tobas_std::isClose(mean_1, mean_2) || !tobas_std::isClose(var_1, var_2))
    {
      cerr << "Moving statistics is inaccurate." << endl;
      return EXIT_FAILURE;
    }

    // Add next data
    moving_stat_.add(incoming_data[i]);
  }

  return EXIT_SUCCESS;
}
