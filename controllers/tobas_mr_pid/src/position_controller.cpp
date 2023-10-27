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

void PositionController::update(
  const Vector& cur_pos,
  const Vector& cur_vel,
  const Vector& tar_pos,
  const Vector& tar_vel,
  Vector& tar_acc,
  const double& dt)
{
  assert(dt >= 0);

  // 誤差を計算
  const auto ep = tar_pos - cur_pos;
  ei_ += ep * dt;
  const auto ed = tar_vel - cur_vel;

  // 目標加速度を計算
  tar_acc.x(cfg_.hor_kp * ep.x() + cfg_.hor_ki * ei_.x() + cfg_.hor_kd * ed.x());
  tar_acc.y(cfg_.hor_kp * ep.y() + cfg_.hor_ki * ei_.y() + cfg_.hor_kd * ed.y());
  tar_acc.z(cfg_.ver_kp * ep.z() + cfg_.ver_ki * ei_.z() + cfg_.ver_kd * ed.z());

  // 目標加速度を制限
  dh_std::clamp2d(tar_acc.x(), tar_acc.y(), cfg_.max_hor_acc);
  tar_acc.z() = clamp(tar_acc.z(), -cfg_.max_ver_acc, cfg_.max_ver_acc);

  // 目標加速度の変化量を制限
  const double max_delta_acc = cfg_.max_jerk * dt;
  const Vector delta_acc = (tar_acc - last_ta_).clamp(-max_delta_acc, max_delta_acc);
  tar_acc = last_ta_ + delta_acc;

  // 最新の目標加速度を更新
  last_ta_ = tar_acc;
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
  assert(cfg.max_jerk >= 0);

  cfg_ = cfg;
}
}  // namespace tobas_mr_pid
