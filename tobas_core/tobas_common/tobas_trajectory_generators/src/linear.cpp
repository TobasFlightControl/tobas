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

TrajectoryPoint LinearSpline::get(const double& _t) const
{
  const auto t = std::clamp(_t, 0., T_);
  return { p0_ + v_ * t, v_, 0. };
}

double LinearSpline::duration() const
{
  return T_;
}
}  // namespace traj
