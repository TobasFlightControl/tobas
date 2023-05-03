#pragma once

#include <Eigen/Core>

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

  void
  update(const Eigen::Vector3d& cur_pos, const Eigen::Vector3d& tar_pos, Eigen::Vector3d& tar_vel);

  void reconfigure(const PositionControllerDynamicParams& params);

private:
  double kp_;
};
}  // namespace tobas_multirotor_controller
