#include "../include/tobas_control/online_trajectory_generator_3d.hpp"

using namespace std;
using namespace Eigen;

namespace ctrl
{
OnlineTrajectoryGenerator3d::OnlineTrajectoryGenerator3d()
{
}

Vector3d OnlineTrajectoryGenerator3d::getCommandPosition() const
{
  return { otg_[0].getCommandPosition(), otg_[1].getCommandPosition(), otg_[2].getCommandPosition() };
}

Vector3d OnlineTrajectoryGenerator3d::getCommandVelocity() const
{
  return { otg_[0].getCommandVelocity(), otg_[1].getCommandVelocity(), otg_[2].getCommandVelocity() };
}

Vector3d OnlineTrajectoryGenerator3d::getCommandAcceleration() const
{
  return { otg_[0].getCommandAcceleration(), otg_[1].getCommandAcceleration(), otg_[2].getCommandAcceleration() };
}

Vector3d OnlineTrajectoryGenerator3d::getTargetPosition() const
{
  return { otg_[0].getTargetPosition(), otg_[1].getTargetPosition(), otg_[2].getTargetPosition() };
}

Vector3d OnlineTrajectoryGenerator3d::getTargetVelocity() const
{
  return { otg_[0].getTargetVelocity(), otg_[1].getTargetVelocity(), otg_[2].getTargetVelocity() };
}

Vector3d OnlineTrajectoryGenerator3d::getTargetAcceleration() const
{
  return { otg_[0].getTargetAcceleration(), otg_[1].getTargetAcceleration(), otg_[2].getTargetAcceleration() };
}

void OnlineTrajectoryGenerator3d::setTargetPosition(const Vector3d& tar_pos)
{
  for (size_t i = 0; i < 3; ++i) {
    otg_[i].setTargetPosition(tar_pos(i));
  }
}

void OnlineTrajectoryGenerator3d::setTargetVelocity(const Vector3d& tar_vel)
{
  for (size_t i = 0; i < 3; ++i) {
    otg_[i].setTargetVelocity(tar_vel(i));
  }
}

void OnlineTrajectoryGenerator3d::setTargetAcceleration(const Vector3d& tar_acc)
{
  for (size_t i = 0; i < 3; ++i) {
    otg_[i].setTargetAcceleration(tar_acc(i));
  }
}

void OnlineTrajectoryGenerator3d::setMinVelocity(size_t idx, double min_vel)
{
  otg_.at(idx).setMinVelocity(min_vel);
}

void OnlineTrajectoryGenerator3d::setMaxVelocity(size_t idx, double max_vel)
{
  otg_.at(idx).setMaxVelocity(max_vel);
}

void OnlineTrajectoryGenerator3d::setMinAcceleration(size_t idx, double min_acc)
{
  otg_.at(idx).setMinAcceleration(min_acc);
}

void OnlineTrajectoryGenerator3d::setMaxAcceleration(size_t idx, double max_acc)
{
  otg_.at(idx).setMaxAcceleration(max_acc);
}

void OnlineTrajectoryGenerator3d::setMaxJerk(size_t idx, double max_jerk)
{
  otg_.at(idx).setMaxJerk(max_jerk);
}

void OnlineTrajectoryGenerator3d::setSpeedOverride(size_t idx, double speed_override)
{
  otg_.at(idx).setSpeedOverride(speed_override);
}

void OnlineTrajectoryGenerator3d::update(
  double dt,
  const Vector3d& cur_pos,
  const Vector3d& cur_vel,
  const Vector3d& cur_acc)
{
  for (size_t i = 0; i < 3; ++i) {
    otg_[i].update(dt, cur_pos(i), cur_vel(i), cur_acc(i));
  }
}

void OnlineTrajectoryGenerator3d::update(double dt)
{
  for (size_t i = 0; i < 3; ++i) {
    otg_[i].update(dt);
  }
}
}  // namespace ctrl
