#pragma once

#include <Eigen/Core>

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

  Eigen::Vector3d update(
    const Eigen::Vector3d& cur_pos,
    const Eigen::Vector3d& cur_vel,
    Eigen::Vector3d tar_pos,
    const Eigen::Vector3d& tar_vel,
    const double& dt);
  void configure(const RotationControllerConfig& config);

private:
  RotationControllerConfig config_;
  Eigen::Vector3d ei_;
};
}  // namespace tobas_mr_pid
