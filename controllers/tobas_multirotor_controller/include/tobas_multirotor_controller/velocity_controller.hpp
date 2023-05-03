#pragma once

#include <Eigen/Core>

namespace tobas_multirotor_controller
{
struct VelocityControllerDynamicParams
{
  double natural_freq;
  double damp_ratio;
};

class VelocityController
{
public:
  explicit VelocityController(const VelocityControllerDynamicParams& params);

  void
  update(const Eigen::Vector3d& cur_vel, const Eigen::Vector3d& tar_vel, Eigen::Vector3d& tar_acc);

  void reconfigure(const VelocityControllerDynamicParams& params);

private:
  double kv_;
};
}  // namespace tobas_multirotor_controller
