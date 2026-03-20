#pragma once

#include <cmath>

namespace traj
{
class VelocityLimitedOnlineTrajectoryGenerator
{
public:
  explicit VelocityLimitedOnlineTrajectoryGenerator();

  inline double getTrajectoryPosition() const;
  inline bool isSaturated() const;

  inline void setTargetPosition(double tar_pos);

  void setMaxVelocity(double max_vel);

  double update(double dt);

  double setTargetPointAndUpdate(double tar_pos, double dt);

  void resetCurrentTrajectoryPoint(double pos);

private:
  // State
  double traj_pos_ = 0.;
  bool is_saturated_ = false;

  // Target
  double tar_pos_ = 0.;

  // Limit
  double max_vel_ = NAN;
};

inline double VelocityLimitedOnlineTrajectoryGenerator::getTrajectoryPosition() const
{
  return traj_pos_;
}

inline bool VelocityLimitedOnlineTrajectoryGenerator::isSaturated() const
{
  return is_saturated_;
}

inline void VelocityLimitedOnlineTrajectoryGenerator::setTargetPosition(double tar_pos)
{
  tar_pos_ = tar_pos;
}
}  // namespace traj
