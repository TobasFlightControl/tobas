#include "../include/tobas_std_tools/algorithm.hpp"
#include "../include/tobas_std_tools/math.hpp"

namespace tobas_std
{
void clamp2d(double& x, double& y, const double& max_length)
{
  assert(max_length >= 0);

  const auto length = sqrt(sqr(x) + sqr(y));
  if (length > max_length)
  {
    const auto scale = max_length / length;
    x *= scale;
    y *= scale;
  }
}



double wrapPi(double angle)
{
  while (angle <= -M_PI)
    angle += 2 * M_PI;
  while (angle > M_PI)
    angle -= 2 * M_PI;
  return angle;
}
}  // namespace tobas_std
