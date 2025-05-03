#include <iostream>

#include "../include/tobas_pose_pid/position_pid.hpp"
#include "./util.hpp"

using namespace std;

namespace tobas
{
PositionPID::PositionPID()
{
  updateGain();
}

kdl::Vector PositionPID::updatePID(
  const kdl::Vector& cur_pos,
  const kdl::Vector& cur_vel,
  const kdl::Vector& tar_pos,
  const kdl::Vector& tar_vel,
  const double& dt)
{
  assert(dt > 0.);

  // 誤差を計算
  const auto ep = tar_pos - cur_pos;
  const auto ed = tar_vel - cur_vel;

  for (size_t i = 0; i < 3; ++i) {
    // I制御を行う場合
    if (ki_(i) > 0.) {
      // 積分誤差を蓄積
      ei_(i) += ep(i) * dt;

      // 最大加速度からP加速度を除いた値でI加速度を制限する (動的なアンチワインドアップ)
      // PI加速度が最大加速度を超える場合，I成分の効果は不安定化とオーバーシュートのみとなってしまう
      const auto tar_acc_p = clamp(kp_(i) * ep(i), -max_acc_(i), max_acc_(i));
      const auto min_ei = (-max_acc_(i) - tar_acc_p) / ki_(i);
      const auto max_ei = (max_acc_(i) - tar_acc_p) / ki_(i);
      ei_(i) = clamp(ei_(i), min_ei, max_ei);
    }
  }

  // 目標加速度を計算
  const auto cmd_acc = kp_.hadamard(ep) + ki_.hadamard(ei_) + kd_.hadamard(ed);

  // 目標加速度を制限して出力
  return cmd_acc.clamp(-max_acc_, max_acc_);
}

kdl::Vector PositionPID::updatePD(
  const kdl::Vector& cur_pos,
  const kdl::Vector& cur_vel,
  const kdl::Vector& tar_pos,
  const kdl::Vector& tar_vel)
{
  // 誤差を計算
  const auto ep = tar_pos - cur_pos;
  const auto ed = tar_vel - cur_vel;

  // 目標加速度を計算
  const auto cmd_acc = kp_.hadamard(ep) + kd_.hadamard(ed);

  // 目標加速度を制限して出力
  return cmd_acc.clamp(-max_acc_, max_acc_);
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
  updateGain();

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
  updateGain();

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
  ei_(idx) = 0.;  // ゲインを変更したら積分誤差をリセット

  return true;
}

bool PositionPID::setMaximumAccel(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value <= 0.) {
    cerr << "Maximum acceleration must be positive." << endl;
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
}  // namespace tobas
