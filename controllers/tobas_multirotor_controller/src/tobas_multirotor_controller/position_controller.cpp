#include <ros/ros.h>

#include "../../include/tobas_multirotor_controller/position_controller.hpp"

using namespace Eigen;

namespace tobas_multirotor_controller
{
PositionController::PositionController(const PositionControllerDynamicParams& params)
{
  reconfigure(params);
}

void PositionController::update(const Vector3d& cur_pos, const Vector3d& tar_pos, Vector3d& tar_vel)
{
  tar_vel = kp_ * (tar_pos - cur_pos);
}

void PositionController::reconfigure(const PositionControllerDynamicParams& params)
{
  ROS_ASSERT(params.natural_freq > 0.);
  ROS_ASSERT(params.damp_ratio > 0.);

  // 速度制御器と位置制御器を合わせると理論的には2次遅れ系の一般系になる (memo: 2-16)
  kp_ = 0.5 * params.natural_freq / params.damp_ratio;
}
}
