#pragma once

#include <kdl/frames.hpp>

namespace tobas_mr_translation_pid
{
struct VelocityControllerConfig
{
  double hor_natural_freq;
  double hor_damp_ratio;
  double ver_natural_freq;
  double ver_damp_ratio;
  double max_hor_vel;
  double max_ver_vel;
};

class VelocityController
{
public:
  explicit VelocityController();

  void update(const KDL::Vector& cur_vel, const KDL::Vector& tar_vel, KDL::Vector& tar_acc);
  void configure(const VelocityControllerConfig& params);

private:
  double hor_kv_;
  double ver_kv_;
  double max_hor_vel_;
  double max_ver_vel_;
};
}  // namespace tobas_mr_translation_pid
