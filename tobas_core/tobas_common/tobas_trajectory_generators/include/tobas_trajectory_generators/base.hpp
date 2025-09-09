#pragma once

namespace traj
{
struct TrajectoryPoint
{
  double p;
  double v;
  double a;
};

/* 軌道生成器の基底クラス */
class TrajectoryGenerator
{
public:
  virtual TrajectoryPoint get(const double& t) const = 0;
  virtual double duration() const = 0;

private:
  double dummy_;
};
}  // namespace traj
