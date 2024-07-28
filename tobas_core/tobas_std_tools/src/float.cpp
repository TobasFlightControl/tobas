#include <cmath>

#include "../include/tobas_std_tools/float.hpp"

using namespace std;

namespace tobas_std
{
bool isClose(const double& x, const double& y, const double& abs_tol, const double& rel_tol)
{
  const auto diff = abs(x - y);
  return diff < abs_tol || diff < rel_tol * max(abs(x), abs(y));
}
}  // namespace tobas_std
