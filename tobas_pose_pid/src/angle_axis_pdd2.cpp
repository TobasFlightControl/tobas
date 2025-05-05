#include "tobas_pose_pid/angle_axis_pdd2.hpp"

#include <iostream>

#include "./util.hpp"

using namespace std;

namespace tobas
{
AngleAxisPDD2::AngleAxisPDD2()
{
  updateGain();
}

kdl::Vector AngleAxisPDD2::update(
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_gyro,
  const kdl::Vector& cur_dgyro,
  const kdl::Rotation& tar_rot,
  const kdl::Vector& tar_gyro,
  const kdl::Vector& tar_dgyro,
  const double& dt)
{
  // Compute error in angle-axis form wrt. the local frame
  const auto ep = (cur_rot.inverse() * tar_rot).getRot();
  const auto ev = tar_gyro - cur_gyro;
  const auto ea = tar_dgyro - cur_dgyro;

  // Compute command ddgyro
  const auto cmd_ddgyro = (kp_.hadamard(ep) + kv_.hadamard(ev) + ka_.hadamard(ea)).clamp(-max_ddgyro_, max_ddgyro_);

  // Integrate command ddgyro
  cmd_dgyro_ += cmd_ddgyro * dt;
  return cmd_dgyro_;
}

bool AngleAxisPDD2::setNaturalFreq(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value <= 0.) {
    cerr << "Natural frequency must be positive." << endl;
    return false;
  }

  wn_(idx) = value;
  updateGain();

  return true;
}

bool AngleAxisPDD2::setInertiaRatio(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value <= 0.) {
    cerr << "Inertia ratio must be positive." << endl;
    return false;
  }

  zeta_(idx) = value;
  updateGain();

  return true;
}

bool AngleAxisPDD2::setDampingRatio(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value < 0.) {
    cerr << "Damping ratio must be non-negative." << endl;
    return false;
  }

  xi_(idx) = value;
  updateGain();

  return true;
}

bool AngleAxisPDD2::setMaximumDDGyro(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value <= 0.) {
    cerr << "Maximum DD-Gyro must be positive." << endl;
    return false;
  }

  max_ddgyro_(idx) = value;

  return true;
}

void AngleAxisPDD2::updateGain()
{
  kp_ = wn_.cube();
  kv_ = 3 * xi_.sqr().hadamard(wn_.sqr());
  ka_ = 3 * zeta_.hadamard(xi_).hadamard(wn_);
}
}  // namespace tobas
