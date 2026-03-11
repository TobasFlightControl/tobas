#pragma once

#include <tobas_trajectory_generators/base.hpp>

namespace tobas
{
namespace mission
{
/* 任意の速度，加速度から最速で停止する軌道 (memo: 3-49) */
class StopTrajectory : public traj::TrajectoryGenerator
{
public:
  explicit StopTrajectory(double p0, double v0, double a0, double am, double jm);

  traj::TrajectoryPoint get(double t) const noexcept override;
  double duration() const noexcept override;

private:
  const double p0_, v0_, a0_;
  double am_, jm_;
  double t1_, t2_, t3_;

  double p(double t) const noexcept;
  double v(double t) const noexcept;
  double a(double t) const noexcept;
};
}  // namespace mission
}  // namespace tobas
