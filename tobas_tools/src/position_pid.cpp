#include <cassert>

#include <tobas_std_tools/algorithm.hpp>
#include <tobas_std_tools/check.hpp>

#include "../include/tobas_tools/position_pid.hpp"

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
  CHECK(cfg.hor_kp > 0);
  CHECK(cfg.hor_ki > 0);
  CHECK(cfg.hor_kd > 0);
  CHECK(cfg.ver_kp > 0);
  CHECK(cfg.ver_ki > 0);
  CHECK(cfg.ver_kd > 0);
  CHECK(cfg.max_hor_acc > 0);
  CHECK(cfg.max_ver_acc > 0);

  kp_.x() = cfg.hor_kp;
  kp_.y() = cfg.hor_kp;
  kp_.z() = cfg.ver_kp;
  ki_.x() = cfg.hor_ki;
  ki_.y() = cfg.hor_ki;
  ki_.z() = cfg.ver_ki;
  kd_.x() = cfg.hor_kd;
  kd_.y() = cfg.hor_kd;
  kd_.z() = cfg.ver_kd;
  max_acc_.x() = cfg.max_hor_acc;
  max_acc_.y() = cfg.max_hor_acc;
  max_acc_.z() = cfg.max_ver_acc;
}
}  // namespace tobas
