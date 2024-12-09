#pragma once

#include <tobas_kdl/vector.hpp>

namespace tobas
{
class PositionPID
{
public:
  explicit PositionPID();

  kdl::Vector update(
    const kdl::Vector& cur_pos,
    const kdl::Vector& cur_vel,
    const kdl::Vector& tar_pos,
    const kdl::Vector& tar_vel,
    const double& dt);

  bool setNaturalFreq(int idx, double value);
  bool setDampingRatio(int idx, double value);
  bool setIntegralGain(int idx, double value);
  bool setMaximumAccel(int idx, double p);

  inline const kdl::Vector& integralError() const;

private:
  // Config
  kdl::Vector natural_freq_ = { 1., 1., 1. };  // [rad/s]
  kdl::Vector damp_ratio_ = { 1., 1., 1. };    // [-]
  kdl::Vector max_acc_ = { 10., 10., 10. };    // [m/s^2]

  // Gain
  kdl::Vector kp_;
  kdl::Vector kd_;
  kdl::Vector ki_ = { 0.1, 0.1, 0.1 };  // [/s^3]

  // Error
  kdl::Vector ei_ = kdl::Vector::Zero();

  void updateGain();
  bool checkIndex(int idx);
};

inline const kdl::Vector& PositionPID::integralError() const
{
  return ei_;
}
}  // namespace tobas
