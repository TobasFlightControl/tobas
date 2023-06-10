#pragma once

#include <kdl/frames.hpp>

namespace tobas_multirotor_controller
{
struct PositionControllerDynamicParams
{
  double natural_freq;
  double damp_ratio;
};

class PositionController
{
public:
  explicit PositionController(const PositionControllerDynamicParams& params);

  void update(const KDL::Vector& cur_pos, const KDL::Vector& tar_pos, KDL::Vector& tar_vel);

  void reconfigure(const PositionControllerDynamicParams& params);

private:
  double kp_;
};
}  // namespace tobas_multirotor_controller
