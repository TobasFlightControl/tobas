#pragma once

#include <Eigen/Core>

namespace tobas
{
struct PositionPidConfig
{
  double hor_natural_freq;  // [rad/s]
  double hor_damp_ratio;    // [-]
  double hor_ki;            // [1/s^3]
  double ver_natural_freq;  // [rad/s]
  double ver_damp_ratio;    // [-]
  double ver_ki;            // [1/s^3]
  double max_hor_acc;       // [m/s^2]
  double max_ver_acc;       // [m/s^2]
};

class PositionPid
{
public:
  explicit PositionPid();

  Eigen::Vector3d update(
    const Eigen::Vector3d& cur_pos,
    const Eigen::Vector3d& cur_vel,
    const Eigen::Vector3d& tar_pos,
    const Eigen::Vector3d& tar_vel,
    const double& dt);

  void configure(const PositionPidConfig& cfg);

  inline const Eigen::Vector3d& integralError() const;

private:
  // Config
  Eigen::Vector3d kp_;
  Eigen::Vector3d ki_;
  Eigen::Vector3d kd_;
  Eigen::Vector3d max_acc_;

  Eigen::Vector3d ei_ = Eigen::Vector3d::Zero();
};

inline const Eigen::Vector3d& PositionPid::integralError() const
{
  return ei_;
}
}  // namespace tobas
