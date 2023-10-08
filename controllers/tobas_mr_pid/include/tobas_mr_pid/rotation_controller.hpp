#pragma once

#include <dh_kdl/frames.hpp>

namespace tobas_mr_pid
{
struct RotationControllerConfig
{
  double atti_kp;
  double atti_ki;
  double atti_kd;
  double head_kp;
  double head_ki;
  double head_kd;

  double max_attitude;
  double max_heading_error;
};

class RotationController
{
public:
  explicit RotationController();

  void update(
    const KDL::Vector& cur_pos,
    const KDL::Vector& cur_vel,
    KDL::Vector tar_pos,
    const KDL::Vector& tar_vel,
    KDL::Vector& tar_acc,
    const double& dt);
  void configure(const RotationControllerConfig& config);

private:
  RotationControllerConfig config_;
  KDL::Vector ei_;
};
}  // namespace tobas_mr_pid
