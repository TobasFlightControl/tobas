#include <cassert>

#include <dh_std_tools/algorithm.hpp>

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

  // 目標加速度を計算
  Vector3d tar_acc = pid_.update(ep, ed, dt);

  // 目標加速度を制限
  dh_std::clamp2d(tar_acc.x(), tar_acc.y(), max_hor_acc_);
  tar_acc.z() = clamp(tar_acc.z(), -max_ver_acc_, max_ver_acc_);

  return tar_acc;
}

void PositionPid::configure(const PositionPidConfig& cfg)
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

  pid_.kp.x() = cfg.hor_kp;
  pid_.kp.y() = cfg.hor_kp;
  pid_.kp.z() = cfg.ver_kp;
  pid_.ki.x() = cfg.hor_ki;
  pid_.ki.y() = cfg.hor_ki;
  pid_.ki.z() = cfg.ver_ki;
  pid_.kd.x() = cfg.hor_kd;
  pid_.kd.y() = cfg.hor_kd;
  pid_.kd.z() = cfg.ver_kd;

  max_hor_acc_ = cfg.max_hor_acc;
  max_ver_acc_ = cfg.max_ver_acc;

  const double max_hor_int_err = cfg.hor_ki > 0 ? cfg.max_hor_acc_int / cfg.hor_ki : 0;
  const double max_ver_int_err = cfg.ver_ki > 0 ? cfg.max_ver_acc_int / cfg.ver_ki : 0;
  pid_.i_max.x() = max_hor_int_err;
  pid_.i_max.y() = max_hor_int_err;
  pid_.i_max.z() = max_ver_int_err;
}
}  // namespace tobas_mr_pid
