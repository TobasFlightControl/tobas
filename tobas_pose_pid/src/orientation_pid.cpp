#include <cassert>

#include <tobas_math/core.hpp>
#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_eigen_tools/geometry.hpp>

#include "../include/tobas_pose_pid/orientation_pid.hpp"

using namespace std;
using namespace Eigen;

namespace tobas
{
OrientationPid::OrientationPid()
{
}

kdl::Vector OrientationPid::update(
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
  const kdl::Vector ed(eigen_tools::eulerrateFromAngvelLocal(gyro_error.data, cur_rpy.roll, cur_rpy.pitch));

  // 積分誤差を蓄積
  // 制御入力の飽和により姿勢が実現できない状況は無いとして，アンチワインドアップは行わない
  ei_ += ep * dt;

  // 目標オイラー角加速度を計算
  const auto tar_euler_acc = kp_.hadamard(ep) + kd_.hadamard(ed) + ki_.hadamard(ei_);

  // オイラー角加速度をDジャイロに変換
  const auto cur_rpyd = eigen_tools::eulerrateFromAngvelLocal(cur_gyro.data, cur_rpy.roll, cur_rpy.pitch);
  return kdl::Vector(eigen_tools::angaccFromEuleraccLocal(cur_rpy.roll, cur_rpy.pitch, cur_rpyd, tar_euler_acc.data));
}

void OrientationPid::configure(const OrientationPidConfig& cfg)
{
  TOBAS_CHECK(cfg.atti_natural_freq > 0);
  TOBAS_CHECK(cfg.atti_damp_ratio > 0);
  TOBAS_CHECK(cfg.atti_ki > 0);
  TOBAS_CHECK(cfg.head_natural_freq > 0);
  TOBAS_CHECK(cfg.head_damp_ratio > 0);
  TOBAS_CHECK(cfg.head_ki > 0);

  const auto atti_kp = math::sqr(cfg.atti_natural_freq);
  const auto atti_kd = 2 * cfg.atti_damp_ratio * cfg.atti_natural_freq;
  const auto head_kp = math::sqr(cfg.head_natural_freq);
  const auto head_kd = 2 * cfg.head_damp_ratio * cfg.head_natural_freq;

  kp_.x() = atti_kp;
  kp_.y() = atti_kp;
  kp_.z() = head_kp;
  ki_.x() = cfg.atti_ki;
  ki_.y() = cfg.atti_ki;
  ki_.z() = cfg.head_ki;
  kd_.x() = atti_kd;
  kd_.y() = atti_kd;
  kd_.z() = head_kd;
}
}  // namespace tobas
