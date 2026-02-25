#pragma once

#include "./base.hpp"

namespace traj
{
/* 直線軌道生成 */
class LinearSpline : public TrajectoryGenerator
{
public:
  explicit LinearSpline(double p0, double pf, double T);

  TrajectoryPoint get(double t) const noexcept override;
  double duration() const noexcept override;

private:
  const double p0_, T_;
  double v_;
};
}  // namespace traj
