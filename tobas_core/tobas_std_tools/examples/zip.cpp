#include <vector>
#include <map>
#include <iostream>

#include <tobas_std_tools/zip.hpp>

using namespace std;

int main()
{
  const vector<double> A = { 0.0, 0.1, 0.2, 0.3 };
  const array<double, 4> B = { 1.0, 1.1, 1.2, 1.3 };
  const double C[4] = { 2.0, 2.1, 2.2, 2.3 };
  const map<string, int> D = { { "a", 1 }, { "b", 2 }, { "c", 3 }, { "d", 4 } };
  const string E = "hoge";

  for (const auto& [a, b, c, d, e] : tobas_std::zip(A, B, C, D, E))
  {
    cout << a << " ";
    cout << b << " ";
    cout << c << " ";
    cout << d.first << " ";
    cout << e << endl;
  }
}
