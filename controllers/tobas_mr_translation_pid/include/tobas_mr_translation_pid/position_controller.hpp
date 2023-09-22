#pragma once

#include <kdl/frames.hpp>

namespace tobas_mr_translation_pid
{
struct PositionControllerConfig
{
  double hor_natural_freq;
  double hor_damp_ratio;
  double ver_natural_freq;
  double ver_damp_ratio;
};

class PositionController
{
public:
  explicit PositionController();

  void update(const KDL::Vector& cur_pos, const KDL::Vector& tar_pos, KDL::Vector& tar_vel);
  void configure(const PositionControllerConfig& params);

private:
  double hor_kp_;
  double ver_kp_;
};
}  // namespace tobas_mr_translation_pid
