#pragma once

namespace traj
{
/* 軌道生成器の基底クラス */
class BaseTrajectory
{
public:
  virtual void get(const double& t, double& p, double& v, double& a) = 0;
  virtual double duration() = 0;
};
}  // namespace traj
