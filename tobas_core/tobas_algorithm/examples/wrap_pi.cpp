#include <cmath>
#include <iostream>

#include <tobas_algorithm/core.hpp>

using namespace std;

int main()
{
  constexpr double angles[] = { -4 * M_PI, -M_PI, -M_PI / 2, 0, M_PI / 2, M_PI, 3 * M_PI };
  for (const auto& angle : angles)
    cout << "Original: " << angle << ", Wrapped: " << algo::wrapPi(angle) << endl;
}
