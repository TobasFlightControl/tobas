#pragma once

#include <cassert>
#include <algorithm>

#include "../include/tobas_mr_pid/rotation_controller.hpp"

using namespace std;
using namespace KDL;

namespace tobas_mr_pid
{
RotationController::RotationController()
{
}

void RotationController::update(
  const Vector& cur_pos,
  const Vector& cur_vel,
  Vector tar_pos,
  const Vector& tar_vel,
  Vector& tar_acc,
  const double& dt)
{
  assert(dt >= 0);

  // 目標姿勢をクランプ
  tar_pos.x(clamp(tar_pos.x(), -config_.max_attitude, config_.max_attitude));
  tar_pos.y(clamp(tar_pos.y(), -config_.max_attitude, config_.max_attitude));
  const auto yaw_error =
    clamp(tar_pos.z() - cur_pos.z(), -config_.max_heading_error, config_.max_heading_error);
  tar_pos.z(cur_pos.z() + yaw_error);

  // 誤差を計算
  const auto ep = tar_pos - cur_pos;
  ei_ += (tar_pos - cur_pos) * dt;
  const auto ed = tar_vel - cur_vel;

  // 目標オイラー加速度を計算
  tar_acc.x(config_.atti_kp * ep.x() + config_.atti_ki * ei_.x() + config_.atti_kd * ed.x());
  tar_acc.y(config_.atti_kp * ep.y() + config_.atti_ki * ei_.y() + config_.atti_kd * ed.y());
  tar_acc.z(config_.head_kp * ep.z() + config_.head_ki * ei_.z() + config_.head_kd * ed.z());
}

void RotationController::configure(const RotationControllerConfig& config)
{
  assert(config.atti_kp >= 0);
  assert(config.atti_ki >= 0);
  assert(config.atti_kd >= 0);
  assert(config.head_kp >= 0);
  assert(config.head_ki >= 0);
  assert(config.head_kd >= 0);

  config_ = config;
}
}  // namespace tobas_mr_pid
