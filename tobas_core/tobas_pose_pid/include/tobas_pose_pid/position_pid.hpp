#pragma once

#include <tobas_kdl/vector.hpp>

namespace tobas
{
class PositionPID
{
public:
  explicit PositionPID();

  kdl::Vector updatePID(
    const kdl::Vector& cur_pos,
    const kdl::Vector& cur_vel,
    const kdl::Vector& tar_pos,
    const kdl::Vector& tar_vel,
    const double& dt);

  kdl::Vector updatePD(
    const kdl::Vector& cur_pos,
    const kdl::Vector& cur_vel,
    const kdl::Vector& tar_pos,
    const kdl::Vector& tar_vel);

  bool setNaturalFreq(int idx, double value);
  bool setDampingRatio(int idx, double value);
  bool setIntegralGain(int idx, double value);
  bool setMaximumAccel(int idx, double value);

  inline const kdl::Vector& getIntegralError() const;
  inline void resetIntegralError();

private:
  // Config
  kdl::Vector natural_freq_ = { 1., 1., 1. };               // [rad/s]
  kdl::Vector damp_ratio_ = { 1., 1., 1. };                 // [-]
  kdl::Vector max_acc_ = { INFINITY, INFINITY, INFINITY };  // [m/s^2]

  // Gain
  kdl::Vector kp_;
  kdl::Vector kd_;
  kdl::Vector ki_ = { 0., 0., 0. };  // [/s^3]

  // Error
  kdl::Vector ei_ = kdl::Vector::Zero();

  void updateGain();
};

inline const kdl::Vector& PositionPID::getIntegralError() const
{
  return ei_;
}

inline void PositionPID::resetIntegralError()
{
  ei_.setZero();
}
}  // namespace tobas
