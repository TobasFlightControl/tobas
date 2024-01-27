#include <cassert>

#include <tobas_std_tools/algorithm.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_eigen_tools/geometry.hpp>

#include "../include/tobas_tools/orientation_pid.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas
{
OrientationPid::OrientationPid()
{
  gyro_lpf_.initialize(tobas_std::timeConstFromCutoffFreq(kGyroLpfCutoff), Vector::Zero());
}

Vector OrientationPid::update(
  const Euler& cur_rpy,
  const Vector& cur_gyro,
  const Euler& tar_rpy,
  const Vector& tar_gyro,
  const double& dt)
{
  // ジャイロノイズ (によるDジャイロのZ成分のノイズ) が回転数に大きく影響するためLPFに通す
  gyro_lpf_.update(cur_gyro, dt);
  const auto& cur_gyro_lpf = gyro_lpf_.getState();

  // 誤差を計算
  // 角軸ベクトルを使うのが正しいが，姿勢角と方位角のゲインを分けるためにオイラー角で計算する
  const Vector ep(
    tar_rpy.roll - cur_rpy.roll, tar_rpy.pitch - cur_rpy.pitch, tar_rpy.yaw - cur_rpy.yaw);
  const Vector gyro_error = tar_gyro - cur_gyro_lpf;
  const Vector ed(
    eigen_tools::eulerrateFromAngvelLocal(gyro_error.data, cur_rpy.roll, cur_rpy.pitch));

  // 積分誤差を蓄積
  // 制御入力の飽和により姿勢が実現できない状況は無いとして，アンチワインドアップは行わない
  ei_ += ep * dt;

  // 目標オイラー角加速度を計算
  const auto tar_euler_acc = kp_.hadamard(ep) + kd_.hadamard(ed) + ki_.hadamard(ei_);

  // オイラー角加速度をDジャイロに変換
  const Vector3d cur_rpyd =
    eigen_tools::eulerrateFromAngvelLocal(cur_gyro_lpf.data, cur_rpy.roll, cur_rpy.pitch);
  return Vector(eigen_tools::angaccFromEuleraccLocal(
    cur_rpy.roll, cur_rpy.pitch, cur_rpyd, tar_euler_acc.data));
}

void OrientationPid::configure(const OrientationPidConfig& cfg)
{
  CHECK(cfg.atti_kp > 0);
  CHECK(cfg.atti_ki > 0);
  CHECK(cfg.atti_kd > 0);
  CHECK(cfg.head_kp > 0);
  CHECK(cfg.head_ki > 0);
  CHECK(cfg.head_kd > 0);

  kp_.x() = cfg.atti_kp;
  kp_.y() = cfg.atti_kp;
  kp_.z() = cfg.head_kp;
  ki_.x() = cfg.atti_ki;
  ki_.y() = cfg.atti_ki;
  ki_.z() = cfg.head_ki;
  kd_.x() = cfg.atti_kd;
  kd_.y() = cfg.atti_kd;
  kd_.z() = cfg.head_kd;
}
}  // namespace tobas
