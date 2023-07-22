#include <ros/ros.h>

#include "../../include/tobas_multirotor_controller/velocity_controller.hpp"

using namespace KDL;

namespace tobas_multirotor_controller
{
VelocityController::VelocityController(const VelocityControllerDynamicParams& params)
{
  reconfigure(params);
}

void VelocityController::update(const Vector& cur_vel, const Vector& tar_vel, Vector& tar_acc)
{
  tar_acc.x(hor_kv_ * (tar_vel.x() - cur_vel.x()));
  tar_acc.y(hor_kv_ * (tar_vel.y() - cur_vel.y()));
  tar_acc.z(ver_kv_ * (tar_vel.z() - cur_vel.z()));
}

void VelocityController::reconfigure(const VelocityControllerDynamicParams& params)
{
  ROS_ASSERT(params.hor_natural_freq > 0.);
  ROS_ASSERT(params.hor_damp_ratio > 0.);
  ROS_ASSERT(params.ver_natural_freq > 0.);
  ROS_ASSERT(params.ver_damp_ratio > 0.);

  // 速度制御器と位置制御器を合わせると理論的には2次遅れ系の一般系になる (memo: 2-16)
  hor_kv_ = 2. * params.hor_natural_freq * params.hor_damp_ratio;
  ver_kv_ = 2. * params.ver_natural_freq * params.ver_damp_ratio;
}
}  // namespace tobas_multirotor_controller
