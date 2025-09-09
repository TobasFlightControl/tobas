#pragma once

#include "./base.hpp"

namespace traj
{
/* 3次多項式軌道生成 (ロボティクス, p.192) */
class CubicSpline : public TrajectoryGenerator
{
public:
  explicit CubicSpline(const double& p0, const double& pf, const double& T);

  TrajectoryPoint get(const double& t) const override;
  double duration() const override;

private:
  const double T_;
  double a0_, a1_, a2_, a3_;
};
}  // namespace traj
