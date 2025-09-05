#include "tobas_trajectory_generators/linear.hpp"

#include <algorithm>
#include <cassert>

namespace traj
{
LinearSpline::LinearSpline(const double& p0, const double& pf, const double& T) : p0_(p0), T_(T)
{
  assert(T > 0);

  v_ = (pf - p0) / T;
}

void LinearSpline::get(const double& _t, double& p, double& v, double& a)
{
  const auto t = std::clamp(_t, 0., T_);
  p = p0_ + v * t;
  v = v_;
  a = 0.;
}

double LinearSpline::duration()
{
  return T_;
}
}  // namespace traj
