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

  // 最大加速度からP加速度を除いた値でI加速度を制限する (動的なアンチワインドアップ)
  // PI加速度が最大加速度を超える場合，I成分の効果は不安定化とオーバーシュートのみとなってしまう
  const Vector3d tar_acc_p = kp_.cwiseProduct(ep).cwiseMax(-max_acc_).cwiseMin(max_acc_);
  const Vector3d min_ei = (-max_acc_ - tar_acc_p).cwiseProduct(ki_.cwiseInverse());
  const Vector3d max_ei = (max_acc_ - tar_acc_p).cwiseProduct(ki_.cwiseInverse());
  ei_ = ei_.cwiseMax(min_ei).cwiseMin(max_ei);

  // PID
  const Vector3d tar_acc = kp_.cwiseProduct(ep) + ki_.cwiseProduct(ei_) + kd_.cwiseProduct(ed);

  // 目標加速度を制限して出力
  return tar_acc.cwiseMax(-max_acc_).cwiseMin(max_acc_);
}

void PositionPid::configure(const PositionPidConfig& cfg)
{
  TOBAS_CHECK(cfg.hor_natural_freq > 0);
  TOBAS_CHECK(cfg.hor_damp_ratio > 0);
  TOBAS_CHECK(cfg.hor_ki > 0);
  TOBAS_CHECK(cfg.ver_natural_freq > 0);
  TOBAS_CHECK(cfg.ver_damp_ratio > 0);
  TOBAS_CHECK(cfg.ver_ki > 0);
  TOBAS_CHECK(cfg.max_hor_acc > 0);
  TOBAS_CHECK(cfg.max_ver_acc > 0);

  const auto hor_kp = math::sqr(cfg.hor_natural_freq);
  const auto hor_kd = 2 * cfg.hor_damp_ratio * cfg.hor_natural_freq;
  const auto ver_kp = math::sqr(cfg.ver_natural_freq);
  const auto ver_kd = 2 * cfg.ver_damp_ratio * cfg.ver_natural_freq;

  kp_.x() = hor_kp;
  kp_.y() = hor_kp;
  kp_.z() = ver_kp;
  ki_.x() = cfg.hor_ki;
  ki_.y() = cfg.hor_ki;
  ki_.z() = cfg.ver_ki;
  kd_.x() = hor_kd;
  kd_.y() = hor_kd;
  kd_.z() = ver_kd;
  max_acc_.x() = cfg.max_hor_acc;
  max_acc_.y() = cfg.max_hor_acc;
  max_acc_.z() = cfg.max_ver_acc;
}
}  // namespace tobas
