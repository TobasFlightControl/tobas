#include "../include/tobas_std_tools/math.hpp"

using namespace std;

namespace tobas_std
{
double wrapPi(double angle)
{
  while (angle <= -M_PI)
    angle += 2 * M_PI;
  while (angle > M_PI)
    angle -= 2 * M_PI;
  return angle;
}

bool isClose(const double& a, const double& b, const double& abs_tol, const double& rel_tol)
{
  const auto diff = abs(a - b);
  if (diff <= abs_tol || diff <= rel_tol * max(abs(a), abs(b)))
  {
    return true;
  }
  return false;
}
}  // namespace tobas_std
