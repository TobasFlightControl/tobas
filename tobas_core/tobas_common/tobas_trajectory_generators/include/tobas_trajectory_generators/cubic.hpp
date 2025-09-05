#pragma once

#include "./base.hpp"

namespace traj
{
/* 3次多項式軌道生成 (ロボティクス, p.192) */
class CubicSpline : public BaseTrajectory
{
public:
  explicit CubicSpline(const double& p0, const double& pf, const double& T);

  void get(const double& t, double& p, double& v, double& a) override;
  double duration() override;

private:
  double T_;
  double a0_, a1_, a2_, a3_;
};
}  // namespace traj
