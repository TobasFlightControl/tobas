#pragma once

#include "./base.hpp"

namespace traj
{
/* 直線軌道生成 */
class LinearSpline : public TrajectoryGenerator
{
public:
  explicit LinearSpline(const double& p0, const double& pf, const double& T);

  TrajectoryPoint get(const double& t) const override;
  double duration() const override;

private:
  const double p0_, T_;
  double v_;
};
}  // namespace traj
