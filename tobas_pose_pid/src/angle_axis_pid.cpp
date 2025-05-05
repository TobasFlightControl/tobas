#include "tobas_pose_pid/angle_axis_pid.hpp"

#include <iostream>

#include "./util.hpp"

using namespace std;

namespace tobas
{
AngleAxisPID::AngleAxisPID()
{
  updateGain();
}

kdl::Vector AngleAxisPID::updatePID(
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_gyro,
  const kdl::Rotation& tar_rot,
  const kdl::Vector& tar_gyro,
  const double& dt)
{
  // Compute error in angle-axis form wrt. the local frame
  const auto ep = (cur_rot.inverse() * tar_rot).getRot();
  const auto ed = tar_gyro - cur_gyro;

  // Integrate error
  for (size_t i = 0; i < 3; ++i) {
    if (ki_(i) > 0.) {
      ei_(i) += ep(i) * dt;
    }
  }

  // Compute target dgyro
  return kp_.hadamard(ep) + ki_.hadamard(ei_) + kd_.hadamard(ed);
}

kdl::Vector AngleAxisPID::updatePD(
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_gyro,
  const kdl::Rotation& tar_rot,
  const kdl::Vector& tar_gyro)
{
  // Compute error in angle-axis form wrt. the local frame
  const auto ep = (cur_rot.inverse() * tar_rot).getRot();
  const auto ed = tar_gyro - cur_gyro;

  // Compute target dgyro
  return kp_.hadamard(ep) + kd_.hadamard(ed);
}

bool AngleAxisPID::setNaturalFreq(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value < 0.) {
    cerr << "Natural frequency must be non-negative." << endl;
    return false;
  }

  natural_freq_(idx) = value;
  updateGain();

  return true;
}

bool AngleAxisPID::setDampingRatio(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value < 0.) {
    cerr << "Damping ratio must be non-negative." << endl;
    return false;
  }

  damp_ratio_(idx) = value;
  updateGain();

  return true;
}

bool AngleAxisPID::setIntegralGain(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value < 0.) {
    cerr << "Integral gain must be non-negative." << endl;
    return false;
  }

  ki_(idx) = value;
  ei_(idx) = 0.;

  return true;
}

void AngleAxisPID::updateGain()
{
  kp_ = natural_freq_.sqr();
  kd_ = 2 * damp_ratio_.hadamard(natural_freq_);
}
}  // namespace tobas
