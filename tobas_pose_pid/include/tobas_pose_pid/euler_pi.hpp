#pragma once

#include <tobas_kdl/euler.hpp>

namespace tobas
{
class EulerPI
{
public:
  explicit EulerPI();

  kdl::Vector update(const kdl::Euler& cur_rpy, const kdl::Euler& tar_rpy, const double& dt);

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

inline const kdl::Vector& EulerPI::getIntegralError() const
{
  return ei_;
}

inline void EulerPI::resetIntegralError()
{
  ei_.setZero();
}
}  // namespace tobas
