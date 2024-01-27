#pragma once

#include <tobas_std_tools/first_order_filter.hpp>
#include <tobas_kdl/euler.hpp>

namespace tobas
{
struct OrientationPidConfig
{
  double atti_kp;
  double atti_ki;
  double atti_kd;
  double head_kp;
  double head_ki;
  double head_kd;
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
