#pragma once

#include <kdl/frames.hpp>

namespace tobas_multirotor_controller
{
struct PositionControllerDynamicParams
{
  double hor_natural_freq;
  double hor_damp_ratio;
  double ver_natural_freq;
  double ver_damp_ratio;
  double max_hor_vel;
  double max_ver_vel;
};

class PositionController
{
public:
  explicit PositionController();

  void update(const KDL::Vector& cur_pos, const KDL::Vector& tar_pos, KDL::Vector& tar_vel);

  void reconfigure(const PositionControllerDynamicParams& params);

private:
  double hor_kp_;
  double ver_kp_;
  double max_hor_vel_;
  double max_ver_vel_;
};
}  // namespace tobas_multirotor_controller
