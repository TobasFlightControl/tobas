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

  KDL::Vector update(
    const KDL::Euler& cur_rpy,
    const KDL::Vector& cur_gyro,
    const KDL::Euler& tar_rpy,
    const KDL::Vector& tar_gyro,
    const double& dt);

  void configure(const OrientationPidConfig& cfg);

  inline KDL::Vector integralError() const;

private:
  // Config
  KDL::Vector kp_;
  KDL::Vector ki_;
  KDL::Vector kd_;

  KDL::Vector ei_ = KDL::Vector::Zero();
  tobas_std::FirstOrderFilter<KDL::Vector> gyro_lpf_;
};

inline KDL::Vector OrientationPid::integralError() const
{
  return ei_;
}
}  // namespace tobas
