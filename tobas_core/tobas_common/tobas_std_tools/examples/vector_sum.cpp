#include <iostream>

#include <tobas_std_tools/vector.hpp>

using namespace std;

int main()
{
  for (size_t n : { 1000000, 10000000, 100000000, 1000000000 }) {
    vector<float> values(n, 1.0);
    const auto naive_sum = tobas::st::sum(values);
    const auto kahan_sum = tobas::st::fsum(values);
    cout << "n = " << n << "\t: Naive Summation = " << naive_sum << ", Kahan Summation = " << kahan_sum << endl;
  }
}
