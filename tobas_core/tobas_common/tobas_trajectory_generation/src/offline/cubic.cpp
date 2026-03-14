#include "tobas_trajectory_generation/offline/cubic.hpp"

#include <algorithm>
#include <stdexcept>

#include <tobas_math/core.hpp>

namespace traj
{
CubicSpline::CubicSpline(double p0, double pf, double T) : T_(T)
{
  a0_ = p0;
  a1_ = 0.;

  if (T > 0.) {
    a2_ = (3 / math::sqr(T)) * (pf - p0);
    a3_ = (-2 / math::cube(T)) * (pf - p0);
  }
  else if (p0 == pf && T == 0.) {
    a2_ = 0.;
    a3_ = 0.;
  }
  else {
    throw std::runtime_error("There is no cubic spline that satisfies the conditions.");
  }
}

TrajectoryPoint CubicSpline::get(double _t) const noexcept
{
  const auto t = std::clamp(_t, 0., T_);
  const auto p = a0_ + a1_ * t + a2_ * math::sqr(t) + a3_ * math::cube(t);
  const auto v = a1_ + 2 * a2_ * t + 3 * a3_ * math::sqr(t);
  const auto a = 2 * a2_ + 6 * a3_ * t;
  return { p, v, a };
}

double CubicSpline::duration() const noexcept
{
  return T_;
}
}  // namespace traj
