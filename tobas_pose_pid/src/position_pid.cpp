#include <iostream>

#include "../include/tobas_pose_pid/position_pid.hpp"

using namespace std;

namespace tobas
{
PositionPID::PositionPID()
{
  updateGain();
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

  // 積分誤差を蓄積
  ei_ += ep * dt;

  // 最大加速度からP加速度を除いた値でI加速度を制限する (動的なアンチワインドアップ)
  // PI加速度が最大加速度を超える場合，I成分の効果は不安定化とオーバーシュートのみとなってしまう
  const kdl::Vector tar_acc_p = kp_.hadamard(ep).clamp(-max_acc_, max_acc_);
  const kdl::Vector min_ei = (-max_acc_ - tar_acc_p).hadamard(ki_.inverse());
  const kdl::Vector max_ei = (max_acc_ - tar_acc_p).hadamard(ki_.inverse());
  ei_ = ei_.clamp(min_ei, max_ei);

  // PID
  const kdl::Vector tar_acc = kp_.hadamard(ep) + ki_.hadamard(ei_) + kd_.hadamard(ed);

  // 目標加速度を制限して出力
  return tar_acc.clamp(-max_acc_, max_acc_);
}

bool PositionPID::setNaturalFreq(int idx, double value)
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

bool PositionPID::setDampingRatio(int idx, double value)
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

bool PositionPID::setIntegralGain(int idx, double value)
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

bool PositionPID::setMaximumAccel(int idx, double value)
{
  if (!checkIndex(idx))
    return false;

  if (value <= 0.)
  {
    cerr << "Maximum horizontal accel must be positive." << endl;
    return false;
  }

  max_acc_(idx) = value;

  return true;
}

void PositionPID::updateGain()
{
  kp_ = natural_freq_.sqr();
  kd_ = 2 * damp_ratio_.hadamard(natural_freq_);
}

bool PositionPID::checkIndex(int idx)
{
  if (idx < 0 || 3 <= idx)
  {
    cerr << "Index " << idx << " is out of range.";
    return false;
  }

  return true;
}
}  // namespace tobas
