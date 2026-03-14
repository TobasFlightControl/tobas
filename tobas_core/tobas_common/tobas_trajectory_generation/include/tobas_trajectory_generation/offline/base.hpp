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
  virtual TrajectoryPoint get(double t) const noexcept = 0;
  virtual double duration() const noexcept = 0;
};
}  // namespace traj
