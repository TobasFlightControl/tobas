#pragma once

#include <kdl/frames.hpp>

namespace tobas_mr_rotation_mpc
{
struct AccelerationControllerDynamicParams
{
  double max_hor_acc;
  double max_ver_acc;
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
  void configure(const AccelerationControllerDynamicParams& params);

private:
  double mass_;

  double max_hor_acc_;
  double max_ver_acc_;
};
}  // namespace tobas_mr_rotation_mpc
