#pragma once

#include <Eigen/Core>

namespace tobas_mr_pid
{
struct OrientationControllerConfig
{
  double atti_kp;
  double atti_ki;
  double atti_kd;
  double head_kp;
  double head_ki;
  double head_kd;
};

class OrientationController
{
public:
  explicit OrientationController();

  Eigen::Vector3d update(
    const Eigen::Vector3d& cur_pos,
    const Eigen::Vector3d& cur_vel,
    const Eigen::Vector3d& tar_pos,
    const Eigen::Vector3d& tar_vel,
    const double& dt);
  void configure(const OrientationControllerConfig& config);

private:
  OrientationControllerConfig config_;
  Eigen::Vector3d ei_;
};
}  // namespace tobas_mr_pid
