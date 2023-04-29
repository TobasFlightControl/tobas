#include <ros/ros.h>

#include "../../include/tobas_multirotor_controller/velocity_controller.hpp"

using namespace Eigen;

VelocityController::VelocityController(const VelocityControllerDynamicParams& params)
{
  reconfigure(params);
}

void VelocityController::update(const Vector3d& cur_vel, const Vector3d& tar_vel, Vector3d& tar_acc)
{
  tar_acc = kv_ * (tar_vel - cur_vel);
}

void VelocityController::reconfigure(const VelocityControllerDynamicParams& params)
{
  ROS_ASSERT(params.natural_freq > 0.);
  ROS_ASSERT(params.damp_ratio > 0.);

  kv_ = 2. * params.natural_freq * params.damp_ratio;
}
