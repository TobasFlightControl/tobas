#include "tobas_std_tools/trajectory.hpp"

#include <algorithm>
#include <cassert>

#include <tobas_math/core.hpp>

using namespace std;

namespace tobas_std
{
CubicSpline::CubicSpline(const double& p0, const double& pf, const double& T)
{
  assert(T > 0);

  T_ = T;
  a0_ = p0;
  a1_ = 0;
  a2_ = (3 / math::sqr(T)) * (pf - p0);
  a3_ = (-2 / math::cube(T)) * (pf - p0);
}

void CubicSpline::get(const double& _t, double& p, double& v, double& a)
{
  const auto t = clamp(_t, 0., T_);
  p = a0_ + a1_ * t + a2_ * math::sqr(t) + a3_ * math::cube(t);
  v = a1_ + 2 * a2_ * t + 3 * a3_ * math::sqr(t);
  a = 2 * a2_ + 6 * a3_ * t;
}

double CubicSpline::duration()
{
  return T_;
}
}  // namespace tobas_std
