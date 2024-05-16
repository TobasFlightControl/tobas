#pragma once

#include <tobas_std_tools/first_order_filter.hpp>
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
  // Constants
  static constexpr size_t kGyroLpfCutoff = 20;

public:
  explicit OrientationPid();

  tobas_kdl::Vector update(
    const tobas_kdl::Euler& cur_rpy,
    const tobas_kdl::Vector& cur_gyro,
    const tobas_kdl::Euler& tar_rpy,
    const tobas_kdl::Vector& tar_gyro,
    const double& dt);

  void configure(const OrientationPidConfig& cfg);

  inline tobas_kdl::Vector integralError() const;

private:
  // Config
  tobas_kdl::Vector kp_;
  tobas_kdl::Vector ki_;
  tobas_kdl::Vector kd_;

  tobas_kdl::Vector ei_ = tobas_kdl::Vector::Zero();
  tobas_std::FirstOrderFilter<tobas_kdl::Vector> gyro_lpf_;
};

inline tobas_kdl::Vector OrientationPid::integralError() const
{
  return ei_;
}
}  // namespace tobas
