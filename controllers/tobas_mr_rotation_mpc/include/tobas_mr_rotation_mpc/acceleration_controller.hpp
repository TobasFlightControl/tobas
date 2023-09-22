#pragma once

#include <kdl/frames.hpp>

namespace tobas_mr_rotation_mpc
{
struct AccelerationControllerConfig
{
  double max_hor_acc;
  double max_ver_acc;
  double max_attitude;
};

class AccelerationController
{
public:
  explicit AccelerationController();

  void update(
    const KDL::Vector& tar_acc,
    const double& yaw,
    double& U_out,
    double& roll_out,
    double& pitch_out);
  void configure(const AccelerationControllerConfig& config);

private:
  double mass_;

  AccelerationControllerConfig config_;
};
}  // namespace tobas_mr_rotation_mpc
