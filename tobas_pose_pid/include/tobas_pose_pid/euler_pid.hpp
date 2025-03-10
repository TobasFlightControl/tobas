#pragma once

#include <tobas_kdl/euler.hpp>

namespace tobas
{
class EulerPID
{
public:
  explicit EulerPID();

  kdl::Vector updatePID(
    const kdl::Euler& cur_rpy,
    const kdl::Vector& cur_gyro,
    const kdl::Euler& tar_rpy,
    const kdl::Vector& tar_gyro,
    const double& dt);

  kdl::Vector updatePD(
    const kdl::Euler& cur_rpy,
    const kdl::Vector& cur_gyro,
    const kdl::Euler& tar_rpy,
    const kdl::Vector& tar_gyro);

  bool setNaturalFreq(int idx, double value);
  bool setDampingRatio(int idx, double value);
  bool setIntegralGain(int idx, double value);

  inline const kdl::Vector& getIntegralError() const;
  inline void resetIntegralError();

private:
  // Config
  kdl::Vector natural_freq_ = { 10., 10., 10. };  // [rad/s]
  kdl::Vector damp_ratio_ = { 1., 1., 1. };       // [-]

  // Gain
  kdl::Vector kp_;
  kdl::Vector kd_;
  kdl::Vector ki_ = { 0., 0., 0. };

  // Error
  kdl::Vector ei_ = kdl::Vector::Zero();

  void updateGain();

  static kdl::Vector computeProportionalError(const kdl::Euler& cur_rpy, const kdl::Euler& tar_rpy);
  static kdl::Vector
  computeDerivativeError(const kdl::Euler& cur_rpy, const kdl::Vector& cur_gyro, const kdl::Vector& tar_gyro);
};

inline const kdl::Vector& EulerPID::getIntegralError() const
{
  return ei_;
}

inline void EulerPID::resetIntegralError()
{
  ei_.setZero();
}
}  // namespace tobas
