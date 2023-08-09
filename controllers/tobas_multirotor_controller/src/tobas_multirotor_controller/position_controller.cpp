#include <dh_std_tools/algorithm.hpp>

#include "../../include/tobas_multirotor_controller/position_controller.hpp"

using namespace std;
using namespace KDL;

namespace tobas_multirotor_controller
{
PositionController::PositionController()
{
}

void PositionController::update(const Vector& cur_pos, const Vector& tar_pos, Vector& tar_vel)
{
  // 水平成分
  auto tar_vx = hor_kp_ * (tar_pos.x() - cur_pos.x());
  auto tar_vy = hor_kp_ * (tar_pos.y() - cur_pos.y());
  dh_std::clamp2d(tar_vx, tar_vy, max_hor_vel_);

  // 垂直成分
  auto tar_vz = ver_kp_ * (tar_pos.z() - cur_pos.z());
  tar_vz = clamp(tar_vz, -max_ver_vel_, max_ver_vel_);

  // 目標加速度を更新
  tar_vel.x(tar_vx);
  tar_vel.y(tar_vy);
  tar_vel.z(tar_vz);
}

void PositionController::reconfigure(const PositionControllerDynamicParams& params)
{
  assert(params.hor_natural_freq > 0.);
  assert(params.hor_damp_ratio > 0.);
  assert(params.ver_natural_freq > 0.);
  assert(params.ver_damp_ratio > 0.);
  assert(params.max_hor_vel > 0.);
  assert(params.max_ver_vel > 0.);

  // 速度制御器と位置制御器を合わせると理論的には2次遅れ系の一般系になる (memo: 2-16)
  hor_kp_ = 0.5 * params.hor_natural_freq / params.hor_damp_ratio;
  ver_kp_ = 0.5 * params.ver_natural_freq / params.ver_damp_ratio;

  max_hor_vel_ = params.max_hor_vel;
  max_ver_vel_ = params.max_ver_vel;
}
}  // namespace tobas_multirotor_controller
