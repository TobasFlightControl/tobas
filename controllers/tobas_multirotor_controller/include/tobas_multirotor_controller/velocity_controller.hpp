#pragma once

#include <kdl/frames.hpp>

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

  void update(const KDL::Vector& cur_vel, const KDL::Vector& tar_vel, KDL::Vector& tar_acc);

  void reconfigure(const VelocityControllerDynamicParams& params);

private:
  double kv_;
};
}  // namespace tobas_multirotor_controller
