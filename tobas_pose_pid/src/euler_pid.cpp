#include <cassert>

#include <tobas_math/core.hpp>
#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_eigen_tools/geometry.hpp>

#include "../include/tobas_pose_pid/euler_pid.hpp"

using namespace std;
using namespace Eigen;

namespace tobas
{
EulerPID::EulerPID()
{
}

kdl::Vector EulerPID::update(
  const kdl::Euler& cur_rpy,
  const kdl::Vector& cur_gyro,
  const kdl::Euler& tar_rpy,
  const kdl::Vector& tar_gyro,
  const double& dt)
{
  // 誤差を計算
  // 角軸ベクトルを使うのが正しいが，姿勢角と方位角のゲインを分けるためにオイラー角で計算する
  const auto roll_err = algo::wrapPi(tar_rpy.roll - cur_rpy.roll);
  const auto pitch_err = algo::wrapPi(tar_rpy.pitch - cur_rpy.pitch);
  const auto yaw_err = algo::wrapPi(tar_rpy.yaw - cur_rpy.yaw);
  const kdl::Vector ep(roll_err, pitch_err, yaw_err);
  const kdl::Vector gyro_error = tar_gyro - cur_gyro;
  const kdl::Vector ed(eigen::eulerrateFromAngvelLocal(gyro_error.data, cur_rpy.roll, cur_rpy.pitch));

  // 積分誤差を蓄積
  // 制御入力の飽和により姿勢が実現できない状況は無いとして，アンチワインドアップは行わない
  ei_ += ep * dt;

  // ゲインを計算
  const kdl::Vector kp = natural_freq_.sqr();
  const kdl::Vector kd = 2 * damp_ratio_.hadamard(natural_freq_);

  // 目標オイラー角加速度を計算
  const auto tar_euler_acc = kp.hadamard(ep) + kd.hadamard(ed) + ki_.hadamard(ei_);

  // オイラー角加速度をDジャイロに変換
  const auto cur_rpyd = eigen::eulerrateFromAngvelLocal(cur_gyro.data, cur_rpy.roll, cur_rpy.pitch);
  return kdl::Vector(eigen::angaccFromEuleraccLocal(cur_rpy.roll, cur_rpy.pitch, cur_rpyd, tar_euler_acc.data));
}

bool EulerPID::setAttitudeNaturalFrequency(double p)
{
  if (p <= 0.)
  {
    cerr << "Attitude natural frequency must be positive." << endl;
    return false;
  }

  natural_freq_.x() = natural_freq_.y() = p;
  return true;
}

bool EulerPID::setAttitudeDampingRatio(double p)
{
  if (p <= 0.)
  {
    cerr << "Attitude damping ratio must be positive." << endl;
    return false;
  }

  damp_ratio_.x() = damp_ratio_.y() = p;
  return true;
}

bool EulerPID::setAttitudeIntegralGain(double p)
{
  if (p <= 0.)
  {
    cerr << "Attitude integral gain must be positive." << endl;
    return false;
  }

  ki_.x() = ki_.y() = p;
  return true;
}

bool EulerPID::setHeadingNaturalFrequency(double p)
{
  if (p <= 0.)
  {
    cerr << "Heading natural frequency must be positive." << endl;
    return false;
  }

  natural_freq_.z() = p;
  return true;
}

bool EulerPID::setHeadingDampingRatio(double p)
{
  if (p <= 0.)
  {
    cerr << "Heading damping ratio must be positive." << endl;
    return false;
  }

  damp_ratio_.z() = p;
  return true;
}

bool EulerPID::setHeadingIntegralGain(double p)
{
  if (p <= 0.)
  {
    cerr << "Heading integral gain must be positive." << endl;
    return false;
  }

  ki_.z() = p;
  return true;
}
}  // namespace tobas
