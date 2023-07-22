#include <ros/ros.h>

#include "../../include/tobas_multirotor_controller/position_controller.hpp"

using namespace KDL;

namespace tobas_multirotor_controller
{
PositionController::PositionController(const PositionControllerDynamicParams& params)
{
  reconfigure(params);
}

void PositionController::update(const Vector& cur_pos, const Vector& tar_pos, Vector& tar_vel)
{
  tar_vel.x(hor_kp_ * (tar_pos.x() - cur_pos.x()));
  tar_vel.y(hor_kp_ * (tar_pos.y() - cur_pos.y()));
  tar_vel.z(ver_kp_ * (tar_pos.z() - cur_pos.z()));
}

void PositionController::reconfigure(const PositionControllerDynamicParams& params)
{
  ROS_ASSERT(params.hor_natural_freq > 0.);
  ROS_ASSERT(params.hor_damp_ratio > 0.);
  ROS_ASSERT(params.ver_natural_freq > 0.);
  ROS_ASSERT(params.ver_damp_ratio > 0.);

  // 速度制御器と位置制御器を合わせると理論的には2次遅れ系の一般系になる (memo: 2-16)
  hor_kp_ = 0.5 * params.hor_natural_freq / params.hor_damp_ratio;
  ver_kp_ = 0.5 * params.ver_natural_freq / params.ver_damp_ratio;
}
}  // namespace tobas_multirotor_controller
