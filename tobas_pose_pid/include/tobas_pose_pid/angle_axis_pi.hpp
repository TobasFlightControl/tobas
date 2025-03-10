#pragma once

#include <tobas_kdl/rotation.hpp>

namespace tobas
{
class AngleAxisPI
{
public:
  explicit AngleAxisPI();

  kdl::Vector update(const kdl::Rotation& cur_rot, const kdl::Rotation& tar_rot, const double& dt);

  bool setProportionalGain(int idx, double value);
  bool setIntegralGain(int idx, double value);

  inline const kdl::Vector& getIntegralError() const;
  inline void resetIntegralError();

private:
  // Gain
  kdl::Vector kp_ = { 0., 0., 0. };
  kdl::Vector ki_ = { 0., 0., 0. };

  // Error
  kdl::Vector ei_ = kdl::Vector::Zero();

  static bool checkIndex(int idx);
};

inline const kdl::Vector& AngleAxisPI::getIntegralError() const
{
  return ei_;
}

inline void AngleAxisPI::resetIntegralError()
{
  ei_.setZero();
}
}  // namespace tobas
