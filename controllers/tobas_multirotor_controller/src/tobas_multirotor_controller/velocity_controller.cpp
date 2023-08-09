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
  // 水平成分
  auto tar_ax = hor_kv_ * (tar_vel.x() - cur_vel.x());
  auto tar_ay = hor_kv_ * (tar_vel.y() - cur_vel.y());
  dh_std::clamp2d(tar_ax, tar_ay, max_hor_acc_);

  // 垂直成分
  auto tar_az = ver_kv_ * (tar_vel.z() - cur_vel.z());
  tar_az = clamp(tar_az, -max_ver_acc_, max_ver_acc_);

  // 目標加速度を更新
  tar_acc.x(tar_ax);
  tar_acc.y(tar_ay);
  tar_acc.z(tar_az);
}

void VelocityController::reconfigure(const VelocityControllerDynamicParams& params)
{
  assert(params.hor_natural_freq > 0.);
  assert(params.hor_damp_ratio > 0.);
  assert(params.ver_natural_freq > 0.);
  assert(params.ver_damp_ratio > 0.);
  assert(params.max_hor_acc > 0.);
  assert(params.max_ver_acc > 0.);

  // 速度制御器と位置制御器を合わせると理論的には2次遅れ系の一般系になる (memo: 2-16)
  hor_kv_ = 2. * params.hor_natural_freq * params.hor_damp_ratio;
  ver_kv_ = 2. * params.ver_natural_freq * params.ver_damp_ratio;

  max_hor_acc_ = params.max_hor_acc;
  max_ver_acc_ = params.max_ver_acc;
}
}  // namespace tobas_multirotor_controller
