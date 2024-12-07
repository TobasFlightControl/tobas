#pragma once

#include <tobas_kdl/euler.hpp>

namespace tobas
{
class EulerPID
{
public:
  explicit EulerPID();

  kdl::Vector update(
    const kdl::Euler& cur_rpy,
    const kdl::Vector& cur_gyro,
    const kdl::Euler& tar_rpy,
    const kdl::Vector& tar_gyro,
    const double& dt);

  bool setAttitudeNaturalFrequency(double p);
  bool setAttitudeDampingRatio(double p);
  bool setAttitudeIntegralGain(double p);
  bool setHeadingNaturalFrequency(double p);
  bool setHeadingDampingRatio(double p);
  bool setHeadingIntegralGain(double p);

  inline kdl::Vector integralError() const;

private:
  // Config
  kdl::Vector natural_freq_ = { 10., 10., 10. };  // [rad/s]
  kdl::Vector damp_ratio_ = { 1., 1., 1. };       // [-]
  kdl::Vector ki_ = { 0.1, 0.1, 0.1 };            // [/s^3]

  kdl::Vector ei_ = kdl::Vector::Zero();
};

inline kdl::Vector EulerPID::integralError() const
{
  return ei_;
}
}  // namespace tobas
