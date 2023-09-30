#pragma once

#include <cassert>

#include <dh_std_tools/algorithm.hpp>

#include "../include/tobas_mr_pid/translation_controller.hpp"

using namespace std;
using namespace KDL;

namespace tobas_mr_pid
{
TranslationController::TranslationController()
{
}

void TranslationController::update(
  const Vector& cur_pos,
  const Vector& cur_vel,
  const Vector& tar_pos,
  Vector tar_vel,
  Vector& tar_acc,
  const double& dt)
{
  assert(dt >= 0);

  // 目標速度をクランプ
  dh_std::clamp2d(tar_vel[0], tar_vel[1], config_.max_hor_vel);
  tar_vel.z(clamp(tar_vel.z(), -config_.max_ver_vel, config_.max_ver_vel));

  // 誤差を計算
  const auto ep = tar_pos - cur_pos;
  ei_ += (tar_pos - cur_pos) * dt;
  const auto ed = cur_vel - tar_vel;

  // 目標加速度を計算
  tar_acc.x(config_.hor_kp * ep.x() + config_.hor_ki * ei_.x() + config_.hor_kd * ed.x());
  tar_acc.y(config_.hor_kp * ep.y() + config_.hor_ki * ei_.y() + config_.hor_kd * ed.y());
  tar_acc.z(config_.ver_kp * ep.z() + config_.ver_ki * ei_.z() + config_.ver_kd * ed.z());
}

void TranslationController::configure(const TranslationControllerConfig& config)
{
  assert(config.hor_kp >= 0);
  assert(config.hor_ki >= 0);
  assert(config.hor_kd >= 0);
  assert(config.ver_kp >= 0);
  assert(config.ver_ki >= 0);
  assert(config.ver_kd >= 0);

  config_ = config;
}
}  // namespace tobas_mr_pid
