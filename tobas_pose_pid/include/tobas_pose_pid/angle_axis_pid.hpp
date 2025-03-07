#pragma once

#include <tobas_kdl/rotation.hpp>

namespace tobas
{
class AngleAxisPID
{
public:
  explicit AngleAxisPID();

  kdl::Vector update(
    const kdl::Rotation& cur_rot,
    const kdl::Vector& cur_gyro,
    const kdl::Rotation& tar_rot,
    const kdl::Vector& tar_gyro,
    const double& dt);

  bool setNaturalFreq(int idx, double value);
  bool setDampingRatio(int idx, double value);
  bool setIntegralGain(int idx, double value);

  inline const kdl::Vector& integralError() const;

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
  bool checkIndex(int idx);
};

inline const kdl::Vector& AngleAxisPID::integralError() const
{
  return ei_;
}
}  // namespace tobas
