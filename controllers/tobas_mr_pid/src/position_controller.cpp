#include <cassert>

#include <dh_std_tools/algorithm.hpp>

#include "../include/tobas_mr_pid/position_controller.hpp"

using namespace std;
using namespace KDL;

namespace tobas_mr_pid
{
PositionController::PositionController()
{
}

Vector PositionController::update(
  const Vector& cur_pos,
  const Vector& cur_vel,
  const Vector& tar_pos,
  const Vector& tar_vel,
  const double& dt)
{
  assert(dt >= 0);

  // 誤差を計算
  const Vector ep = tar_pos - cur_pos;
  ei_ += ep * dt;
  const Vector ed = tar_vel - cur_vel;

  // アンチワインドアップ
  dh_std::clamp2d(ei_.x(), ei_.y(), max_hor_int_err_);
  ei_.z(clamp(ei_.z(), -max_ver_int_err_, max_ver_int_err_));

  // 目標加速度を計算
  Vector tar_acc = kp_.hadamard(ep) + ki_.hadamard(ei_) + kd_.hadamard(ed);

  // 目標加速度を制限
  dh_std::clamp2d(tar_acc.x(), tar_acc.y(), max_hor_acc_);
  tar_acc.z(clamp(tar_acc.z(), -max_ver_acc_, max_ver_acc_));

  return tar_acc;
}

void PositionController::configure(const PositionControllerConfig& cfg)
{
  assert(cfg.hor_kp >= 0);
  assert(cfg.hor_ki >= 0);
  assert(cfg.hor_kd >= 0);
  assert(cfg.ver_kp >= 0);
  assert(cfg.ver_ki >= 0);
  assert(cfg.ver_kd >= 0);
  assert(cfg.max_hor_acc >= 0);
  assert(cfg.max_ver_acc >= 0);
  assert(cfg.max_hor_acc_int >= 0);
  assert(cfg.max_ver_acc_int >= 0);

  kp_.x(cfg.hor_kp);
  kp_.y(cfg.hor_kp);
  kp_.z(cfg.ver_kp);
  ki_.x(cfg.hor_ki);
  ki_.y(cfg.hor_ki);
  ki_.z(cfg.ver_ki);
  kd_.x(cfg.hor_kd);
  kd_.y(cfg.hor_kd);
  kd_.z(cfg.ver_kd);

  max_hor_acc_ = cfg.max_hor_acc;
  max_ver_acc_ = cfg.max_ver_acc;

  max_hor_int_err_ = cfg.hor_ki > 0 ? cfg.max_hor_acc_int / cfg.hor_ki : 0;
  max_ver_int_err_ = cfg.ver_ki > 0 ? cfg.max_ver_acc_int / cfg.ver_ki : 0;
}
}  // namespace tobas_mr_pid
