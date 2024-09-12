#pragma once

#include <eigen3/Eigen/Core>

namespace tobas
{
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

  bool setHorizontalNaturalFrequency(double p);
  bool setHorizontalDampingRatio(double p);
  bool setHorizontalIntegralGain(double p);
  bool setVerticalNaturalFrequency(double p);
  bool setVerticalDampingRatio(double p);
  bool setVerticalIntegralGain(double p);
  bool setMaximumHorizontalAccel(double p);
  bool setMaximumVerticalAccel(double p);

  inline const Eigen::Vector3d& integralError() const;

private:
  // Config
  Eigen::Vector3d natural_freq_ = { 1., 1., 1. };  // [rad/s]
  Eigen::Vector3d damp_ratio_ = { 1., 1., 1. };    // [-]
  Eigen::Vector3d ki_ = { 0.1, 0.1, 0.1 };         // [/s^3]
  Eigen::Vector3d max_acc_ = { 10., 10., 10. };    // [m/s^2]

  Eigen::Vector3d ei_ = Eigen::Vector3d::Zero();
};

inline const Eigen::Vector3d& PositionPid::integralError() const
{
  return ei_;
}
}  // namespace tobas
