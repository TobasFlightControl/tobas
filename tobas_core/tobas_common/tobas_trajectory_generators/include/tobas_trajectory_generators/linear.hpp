#pragma once

#include "./base.hpp"

namespace traj
{
/* 直線軌道生成 */
class LinearSpline : public BaseTrajectory
{
public:
  explicit LinearSpline(const double& p0, const double& pf, const double& T);

  void get(const double& t, double& p, double& v, double& a) override;
  double duration() override;

  using BaseTrajectory::get;

private:
  const double p0_, T_;
  double v_;
};
}  // namespace traj
