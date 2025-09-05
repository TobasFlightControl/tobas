#pragma once

namespace traj
{
/* 軌道生成器の基底クラス */
class BaseTrajectory
{
public:
  virtual void get(const double& t, double& p, double& v, double& a) = 0;
  virtual double duration() = 0;

  virtual void get(const double& t, double& p);
  virtual void get(const double& t, double& p, double& v);

private:
  double dummy_;
};
}  // namespace traj
