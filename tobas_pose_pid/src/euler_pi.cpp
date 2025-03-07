#include <tobas_algorithm/core.hpp>
#include <tobas_eigen_tools/geometry.hpp>

#include "../include/tobas_pose_pid/euler_pi.hpp"

using namespace std;

namespace tobas
{
EulerPI::EulerPI()
{
}

kdl::Vector EulerPI::update(const kdl::Euler& cur_rpy, const kdl::Euler& tar_rpy, const double& dt)
{
  assert(dt > 0.);

  // 誤差を計算
  // XXX: 2つのオイラー角を結ぶ直線は回転における最短距離ではないことに注意
  const auto roll_err = algo::wrapPi(tar_rpy.roll - cur_rpy.roll);
  const auto pitch_err = algo::wrapPi(tar_rpy.pitch - cur_rpy.pitch);
  const auto yaw_err = algo::wrapPi(tar_rpy.yaw - cur_rpy.yaw);
  const kdl::Vector ep(roll_err, pitch_err, yaw_err);

  // I制御を行う場合は積分誤差を蓄積
  for (size_t i = 0; i < 3; ++i)
    if (ki_(i) > 0.)
      ei_(i) += ep(i) * dt;

  // 目標オイラー角速度を計算
  const auto tar_drpy = kp_.hadamard(ep) + ki_.hadamard(ei_);

  // オイラー角速度をジャイロに変換
  return eigen::angvelFromEulerrateLocal(tar_drpy.data, cur_rpy.roll, cur_rpy.pitch);
}

bool EulerPI::setProportionalGain(int idx, double value)
{
  if (!checkIndex(idx))
    return false;

  if (value < 0.)
  {
    cerr << "Proportional gain must be non-negative." << endl;
    return false;
  }

  kp_(idx) = value;

  return true;
}

bool EulerPI::setIntegralGain(int idx, double value)
{
  if (!checkIndex(idx))
    return false;

  if (value < 0.)
  {
    cerr << "Integral gain must be non-negative." << endl;
    return false;
  }

  ki_(idx) = value;
  ei_(idx) = 0.;

  return true;
}

bool EulerPI::checkIndex(int idx)
{
  if (idx < 0 || 3 <= idx)
  {
    cerr << "Index " << idx << " is out of range.";
    return false;
  }

  return true;
}
}  // namespace tobas
