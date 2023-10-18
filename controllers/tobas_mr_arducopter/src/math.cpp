#include <cassert>

#include "../include/tobas_mr_arducopter/math.hpp"

namespace tobas_mr_arducopter
{
double radians(double deg)
{
  return deg * M_PI / 180;
}

double degrees(double rad)
{
  return rad * 180 / M_PI;
}

double calcLowPassAlphaDt(double dt, double cutoff_freq)
{
  assert(cutoff_freq >= 0 && dt >= 0);

  if (cutoff_freq == 0)
  {
    return 1;
  }
  if (dt == 0)
  {
    return 0;
  }

  const double rc = 1 / (M_2PI * cutoff_freq);
  return dt / (dt + rc);
}

double wrap_PI(const double radian)
{
  double res = wrap_2PI(radian);
  if (res > M_PI)
  {
    res -= M_2PI;
  }
  return res;
}

double wrap_2PI(const double radian)
{
  double res = fmod(radian, M_2PI);
  if (res < 0)
  {
    res += M_2PI;
  }
  return res;
}
}  // namespace tobas_mr_arducopter
