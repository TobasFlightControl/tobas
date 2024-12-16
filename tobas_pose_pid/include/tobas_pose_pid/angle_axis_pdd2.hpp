#pragma once

#include <tobas_kdl/rotation.hpp>

namespace tobas
{
class AngleAxisPDD2
{
public:
  explicit AngleAxisPDD2();

  kdl::Vector update(
    const kdl::Rotation& cur_rot,
    const kdl::Vector& cur_gyro,
    const kdl::Vector& cur_dgyro,
    const kdl::Rotation& tar_rot,
    const kdl::Vector& tar_gyro,
    const kdl::Vector& tar_dgyro,
    const double& dt);

  bool setNaturalFreq(int idx, double value);
  bool setInertiaRatio(int idx, double value);
  bool setDampingRatio(int idx, double value);
  bool setMaximumDDGyro(int idx, double value);

private:
  // Config
  kdl::Vector wn_ = { 10., 10., 10. };             // [rad/s]
  kdl::Vector zeta_ = { 1., 1., 1. };              // [-]
  kdl::Vector xi_ = { 1., 1., 1. };                // [-]
  kdl::Vector max_ddgyro_ = { 100., 100., 100. };  // [rad/s^3]

  // Gain
  kdl::Vector kp_;
  kdl::Vector kv_;
  kdl::Vector ka_;

  kdl::Vector cmd_dgyro_ = kdl::Vector::Zero();

  void updateGain();
  bool checkIndex(int idx);
};
}  // namespace tobas
