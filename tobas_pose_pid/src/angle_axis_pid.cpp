#include <iostream>

#include "../include/tobas_pose_pid/angle_axis_pid.hpp"

using namespace std;

namespace tobas
{
AngleAxisPID::AngleAxisPID()
{
  updateGain();
}

kdl::Vector AngleAxisPID::update(
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_gyro,
  const kdl::Rotation& tar_rot,
  const kdl::Vector& tar_gyro,
  const double& dt)
{
  // Compute error in angle-axis form
  const auto ep = (cur_rot.inverse() * tar_rot).getRot();
  const auto ed = tar_gyro - cur_gyro;

  // Integrate error
  ei_ += ep * dt;

  // Compute target dgyro
  return kp_.hadamard(ep) + kd_.hadamard(ed) + ki_.hadamard(ei_);
}

bool AngleAxisPID::setNaturalFreq(int idx, double value)
{
  if (!checkIndex(idx))
    return false;

  if (value <= 0.)
  {
    cerr << "Natural frequency must be positive." << endl;
    return false;
  }

  natural_freq_(idx) = value;
  updateGain();

  return true;
}

bool AngleAxisPID::setDampingRatio(int idx, double value)
{
  if (!checkIndex(idx))
    return false;

  if (value <= 0.)
  {
    cerr << "Damping ratio must be positive." << endl;
    return false;
  }

  damp_ratio_(idx) = value;
  updateGain();

  return true;
}

bool AngleAxisPID::setIntegralGain(int idx, double value)
{
  if (!checkIndex(idx))
    return false;

  if (value <= 0.)
  {
    cerr << "Integral gain must be positive." << endl;
    return false;
  }

  ki_(idx) = value;

  return true;
}

void AngleAxisPID::updateGain()
{
  kp_ = natural_freq_.sqr();
  kd_ = 2 * damp_ratio_.hadamard(natural_freq_);
}

bool AngleAxisPID::checkIndex(int idx)
{
  if (idx < 0 || 3 <= idx)
  {
    cerr << "Index " << idx << " is out of range.";
    return false;
  }

  return true;
}
}  // namespace tobas
