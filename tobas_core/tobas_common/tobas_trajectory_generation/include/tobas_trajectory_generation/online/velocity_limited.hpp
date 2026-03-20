#pragma once

#include <cmath>

namespace traj
{
class VelocityLimitedOnlineTrajectoryGenerator
{
public:
  explicit VelocityLimitedOnlineTrajectoryGenerator();

  inline double getTrajectoryPosition() const;

  inline void setTargetPosition(double tar_pos);

  void setMaxVelocity(double max_vel);

  void update(double dt);

  void setTargetPointAndUpdate(double tar_pos, double dt);

  void resetCurrentTrajectoryPoint(double pos);

private:
  // Trajectory Point
  double traj_pos_ = 0.;

  // Target
  double tar_pos_ = 0.;

  // Limit
  double max_vel_ = NAN;
};

inline double VelocityLimitedOnlineTrajectoryGenerator::getTrajectoryPosition() const
{
  return traj_pos_;
}

inline void VelocityLimitedOnlineTrajectoryGenerator::setTargetPosition(double tar_pos)
{
  tar_pos_ = tar_pos;
}
}  // namespace traj
