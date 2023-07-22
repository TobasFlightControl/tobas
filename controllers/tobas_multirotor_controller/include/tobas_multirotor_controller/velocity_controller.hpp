#pragma once

#include <kdl/frames.hpp>

namespace tobas_multirotor_controller
{
struct VelocityControllerDynamicParams
{
  double hor_natural_freq;
  double hor_damp_ratio;
  double ver_natural_freq;
  double ver_damp_ratio;
};

class VelocityController
{
public:
  explicit VelocityController();

  void update(const KDL::Vector& cur_vel, const KDL::Vector& tar_vel, KDL::Vector& tar_acc);

  void reconfigure(const VelocityControllerDynamicParams& params);

private:
  double hor_kv_;
  double ver_kv_;
};
}  // namespace tobas_multirotor_controller
