#include <dh_std_tools/algorithm.hpp>

#include "../../include/tobas_multirotor_controller/velocity_controller.hpp"

using namespace std;
using namespace KDL;

namespace tobas_multirotor_controller
{
VelocityController::VelocityController()
{
}

void VelocityController::update(const Vector& cur_vel, const Vector& tar_vel, Vector& tar_acc)
{
  // 目標速度を制限
  // 位置制御，速度制御の両方において速度を制限するために，位置制御器の最後ではなくここでクランプする
  auto tar_vx = tar_vel.x();
  auto tar_vy = tar_vel.y();
  dh_std::clamp2d(tar_vx, tar_vy, max_hor_vel_);
  const auto tar_vz = clamp(tar_vel.z(), -max_ver_vel_, max_ver_vel_);

  // 目標加速度を更新
  tar_acc.x(hor_kv_ * (tar_vx - cur_vel.x()));
  tar_acc.y(hor_kv_ * (tar_vy - cur_vel.y()));
  tar_acc.z(ver_kv_ * (tar_vz - cur_vel.z()));
}

void VelocityController::reconfigure(const VelocityControllerDynamicParams& params)
{
  assert(params.hor_natural_freq > 0.);
  assert(params.hor_damp_ratio > 0.);
  assert(params.ver_natural_freq > 0.);
  assert(params.ver_damp_ratio > 0.);
  assert(params.max_hor_vel > 0.);
  assert(params.max_ver_vel > 0.);

  // 速度制御器と位置制御器を合わせると理論的には2次遅れ系の一般系になる (memo: 2-16)
  hor_kv_ = 2. * params.hor_natural_freq * params.hor_damp_ratio;
  ver_kv_ = 2. * params.ver_natural_freq * params.ver_damp_ratio;

  max_hor_vel_ = params.max_hor_vel;
  max_ver_vel_ = params.max_ver_vel;
}
}  // namespace tobas_multirotor_controller
