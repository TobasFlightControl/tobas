#include <random>

#include <tobas_std_tools/vector.hpp>
#include <tobas_std_tools/float.hpp>

#include <tobas_dsp/welford.hpp>

using namespace std;

int main()
{
  // Define random generator
  random_device rnd_dev;
  mt19937 rnd_gen(rnd_dev());
  uniform_real_distribution<double> uniform(0., 1.);

  // Create data
  vector<double> data;
  for (size_t _ = 0; _ < 10; ++_)
    data.push_back(uniform(rnd_gen));
  cout << "Data: " << data << endl;

  // Compute variance with normal method
  const auto mean_1 = tobas_std::fmean(data);
  const auto var_1 = tobas_std::variance(data);

  // Compute variance with Welford method
  dsp::Welford welford;
  for (const auto& x : data)
    welford.add(x);
  const auto mean_2 = welford.mean();
  const auto var_2 = welford.variance();

  // Show results
  cout << "Mean (Normal Method)     :" << mean_1 << endl;
  cout << "Mean (Welford Method)    :" << mean_2 << endl;
  cout << "Variance (Normal Method) : " << var_1 << endl;
  cout << "Variance (Welford Method): " << var_2 << endl;

  // Validate
  if (!tobas_std::isClose(mean_1, mean_2) || !tobas_std::isClose(var_1, var_2))
  {
    cerr << "Welford method failed." << endl;
    return 1;
  }

  return 0;
}
