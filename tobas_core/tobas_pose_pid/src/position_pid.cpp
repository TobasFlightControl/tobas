#include "tobas_pose_pid/position_pid.hpp"

#include <iostream>

#include "./util.hpp"

using namespace std;

namespace tobas
{
PositionPID::PositionPID()
{
}

kdl::Vector PositionPID::update(
  const kdl::Vector& cur_pos,
  const kdl::Vector& cur_vel,
  const kdl::Vector& tar_pos,
  const kdl::Vector& tar_vel,
  const double& dt)
{
  // 誤差を計算
  const auto ep = tar_pos - cur_pos;
  const auto ed = tar_vel - cur_vel;

  for (size_t i = 0; i < 3; ++i) {
    if (ki_(i) > 0.) {  // I制御を行う場合
      // 積分誤差を蓄積
      const auto next_ei = ei_(i) + ep(i) * dt;
      ei_(i) = clamp(next_ei, -max_i_acc_(i), max_i_acc_(i));
    }
    else  // I制御を行わない場合
    {
      // 積分誤差をリセット
      ei_(i) = 0.;
    }
  }

  // 目標加速度を計算
  return kp_.hadamard(ep) + ki_.hadamard(ei_) + kd_.hadamard(ed);
}

bool PositionPID::setProportionalGain(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value < 0.) {
    cerr << "Proportional gain must be non-negative." << endl;
    return false;
  }

  kp_(idx) = value;

  return true;
}

bool PositionPID::setIntegralGain(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value < 0.) {
    cerr << "Integral gain must be non-negative." << endl;
    return false;
  }

  ki_(idx) = value;

  return true;
}

bool PositionPID::setDerivativeGain(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value < 0.) {
    cerr << "Derivative gain must be non-negative." << endl;
    return false;
  }

  kd_(idx) = value;

  return true;
}

bool PositionPID::setNaturalFreq(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value < 0.) {
    cerr << "Natural frequency must be non-negative." << endl;
    return false;
  }

  natural_freq_(idx) = value;
  setGainFromSecondOrderFrom();

  return true;
}

bool PositionPID::setDampingRatio(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value < 0.) {
    cerr << "Damping ratio must be non-negative." << endl;
    return false;
  }

  damp_ratio_(idx) = value;
  setGainFromSecondOrderFrom();

  return true;
}

bool PositionPID::setMaxIntegralAccel(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value <= 0.) {
    cerr << "Maximum acceleration must be positive." << endl;
    return false;
  }

  max_i_acc_(idx) = value;

  return true;
}

void PositionPID::setGainFromSecondOrderFrom()
{
  kp_ = natural_freq_.sqr();
  kd_ = 2 * damp_ratio_.hadamard(natural_freq_);
}
}  // namespace tobas
