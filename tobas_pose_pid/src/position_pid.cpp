#include <cassert>

#include <tobas_math/core.hpp>
#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/check.hpp>

#include "../include/tobas_pose_pid/position_pid.hpp"

using namespace std;
using namespace Eigen;

namespace tobas
{
PositionPid::PositionPid()
{
}

Vector3d PositionPid::update(
  const Vector3d& cur_pos,
  const Vector3d& cur_vel,
  const Vector3d& tar_pos,
  const Vector3d& tar_vel,
  const double& dt)
{
  // 誤差を計算
  const auto ep = tar_pos - cur_pos;
  const auto ed = tar_vel - cur_vel;

  // 積分誤差を蓄積
  ei_ += ep * dt;

  // ゲインを計算
  const Vector3d kp = natural_freq_.cwiseAbs2();
  const Vector3d kd = 2 * damp_ratio_.cwiseProduct(natural_freq_);

  // 最大加速度からP加速度を除いた値でI加速度を制限する (動的なアンチワインドアップ)
  // PI加速度が最大加速度を超える場合，I成分の効果は不安定化とオーバーシュートのみとなってしまう
  const Vector3d tar_acc_p = kp.cwiseProduct(ep).cwiseMax(-max_acc_).cwiseMin(max_acc_);
  const Vector3d min_ei = (-max_acc_ - tar_acc_p).cwiseProduct(ki_.cwiseInverse());
  const Vector3d max_ei = (max_acc_ - tar_acc_p).cwiseProduct(ki_.cwiseInverse());
  ei_ = ei_.cwiseMax(min_ei).cwiseMin(max_ei);

  // PID
  const Vector3d tar_acc = kp.cwiseProduct(ep) + ki_.cwiseProduct(ei_) + kd.cwiseProduct(ed);

  // 目標加速度を制限して出力
  return tar_acc.cwiseMax(-max_acc_).cwiseMin(max_acc_);
}

bool PositionPid::setHorizontalNaturalFrequency(double p)
{
  if (p <= 0.)
  {
    cerr << "Horizontal natural frequency must be positive." << endl;
    return false;
  }

  natural_freq_.x() = natural_freq_.y() = p;
  return true;
}

bool PositionPid::setHorizontalDampingRatio(double p)
{
  if (p <= 0.)
  {
    cerr << "Horizontal damping ratio must be positive." << endl;
    return false;
  }

  damp_ratio_.x() = damp_ratio_.y() = p;
  return true;
}

bool PositionPid::setHorizontalIntegralGain(double p)
{
  if (p <= 0.)
  {
    cerr << "Horizontal integral gain must be positive." << endl;
    return false;
  }

  ki_.x() = ki_.y() = p;
  return true;
}

bool PositionPid::setVerticalNaturalFrequency(double p)
{
  if (p <= 0.)
  {
    cerr << "Vertical natural frequency must be positive." << endl;
    return false;
  }

  natural_freq_.z() = p;
  return true;
}

bool PositionPid::setVerticalDampingRatio(double p)
{
  if (p <= 0.)
  {
    cerr << "Vertical damping ratio must be positive." << endl;
    return false;
  }

  damp_ratio_.z() = p;
  return true;
}

bool PositionPid::setVerticalIntegralGain(double p)
{
  if (p <= 0.)
  {
    cerr << "Vertical integral gain must be positive." << endl;
    return false;
  }

  ki_.z() = p;
  return true;
}

bool PositionPid::setMaximumHorizontalAccel(double p)
{
  if (p <= 0.)
  {
    cerr << "Maximum horizontal accel must be positive." << endl;
    return false;
  }

  max_acc_.x() = max_acc_.y() = p;
  return true;
}

bool PositionPid::setMaximumVerticalAccel(double p)
{
  if (p <= 0.)
  {
    cerr << "Maximum vertical accel must be positive." << endl;
    return false;
  }

  max_acc_.z() = p;
  return true;
}
}  // namespace tobas
