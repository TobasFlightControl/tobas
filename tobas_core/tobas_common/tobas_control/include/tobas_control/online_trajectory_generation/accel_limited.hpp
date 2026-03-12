#pragma once

#include <cmath>

namespace ctrl
{
/* memo: 3-50 */
class AccelLimitedOnlineTrajectoryGenerator
{
public:
  explicit AccelLimitedOnlineTrajectoryGenerator();

  inline double getTrajectoryPosition() const;
  inline double getTrajectoryVelocity() const;

  void setTargetPosition(double tar_pos);
  void setTargetVelocity(double tar_vel);

  void setMaxVelocity(double max_vel);
  void setMaxAccel(double max_acc);

  void update(double dt);

  void resetCurrentTrajectoryPoint(double pos, double vel);

private:
  enum State
  {
    kFirstBang,
    kSecondBang,
    kDone,
  } state_;

  // Trajectory Point
  double p_ = 0.;
  double v_ = 0.;

  // Target
  double pf_ = 0.;
  double vf_ = 0.;

  // Limit
  double vm_ = NAN;
  double am_ = NAN;

  // Command
  double sign_ = 0.;

  double switchingCurve() const;
  int controlSign() const;
  bool hasCrossedSwitchingCurve() const;

  bool isCloseToTarget(double dt);
  void fixToTarget();

  void step(double dt);
};

inline double AccelLimitedOnlineTrajectoryGenerator::getTrajectoryPosition() const
{
  return p_;
}

inline double AccelLimitedOnlineTrajectoryGenerator::getTrajectoryVelocity() const
{
  return v_;
}
}  // namespace ctrl
