#pragma once

#include <tobas_kdl/euler.hpp>

namespace tobas
{
struct OrientationPidConfig
{
  double atti_natural_freq;  // [rad/s]
  double atti_damp_ratio;    // [-]
  double atti_ki;            // [1/s^3]
  double head_natural_freq;  // [rad/s]
  double head_damp_ratio;    // [-]
  double head_ki;            // [1/s^3]
};

class OrientationPid
{
public:
  explicit OrientationPid();

  kdl::Vector update(
    const kdl::Euler& cur_rpy,
    const kdl::Vector& cur_gyro,
    const kdl::Euler& tar_rpy,
    const kdl::Vector& tar_gyro,
    const double& dt);

  void configure(const OrientationPidConfig& cfg);

  inline kdl::Vector integralError() const;

private:
  // Config
  kdl::Vector kp_;
  kdl::Vector ki_;
  kdl::Vector kd_;

  kdl::Vector ei_ = kdl::Vector::Zero();
};

inline kdl::Vector OrientationPid::integralError() const
{
  return ei_;
}
}  // namespace tobas
