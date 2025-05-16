#include "tobas_pose_pid/position_pdd2.hpp"

#include <iostream>

#include "./util.hpp"

using namespace std;

namespace tobas
{
PositionPDD2::PositionPDD2()
{
  updateGain();
}

kdl::Vector PositionPDD2::update(
  const kdl::Vector& cur_pos,
  const kdl::Vector& cur_vel,
  const kdl::Vector& cur_acc,
  const kdl::Vector& tar_pos,
  const kdl::Vector& tar_vel,
  const kdl::Vector& tar_acc,
  const double& dt)
{
  // 誤差を計算
  const auto ep = tar_pos - cur_pos;
  const auto ev = tar_vel - cur_vel;
  const auto ea = tar_acc - cur_acc;

  // PDD2 + clamp
  const auto cmd_jerk = (kp_.hadamard(ep) + kv_.hadamard(ev) + ka_.hadamard(ea)).clamp(-max_jerk_, max_jerk_);

  // 目標ジャークを積分して出力
  cmd_acc_ += cmd_jerk * dt;
  return cmd_acc_;
}

bool PositionPDD2::setNaturalFreq(int idx, double value)
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

bool PositionPDD2::setInertiaRatio(int idx, double value)
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

bool PositionPDD2::setDampingRatio(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value <= 0.) {
    cerr << "Damping ratio must be positive." << endl;
    return false;
  }

  xi_(idx) = value;
  updateGain();

  return true;
}

bool PositionPDD2::setMaximumJerk(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value <= 0.) {
    cerr << "Maximum jerk must be positive." << endl;
    return false;
  }

  max_jerk_(idx) = value;

  return true;
}

void PositionPDD2::updateGain()
{
  kp_ = wn_.cube();
  kv_ = 3 * xi_.sqr().hadamard(wn_.sqr());
  ka_ = 3 * zeta_.hadamard(xi_).hadamard(wn_);
}
}  // namespace tobas
