#pragma once

#include <eigen3/Eigen/Core>

namespace ctrl
{
class PID3
{
public:
  Eigen::Vector3d kp;
  Eigen::Vector3d ki;
  Eigen::Vector3d kd;
  Eigen::Vector3d i_max;

  explicit PID3();

  Eigen::Vector3d update(const Eigen::Vector3d& ep, const Eigen::Vector3d& ed, const double& dt);

  inline void reset();

  inline const Eigen::Vector3d& integralError() const;

private:
  Eigen::Vector3d ei_;
};

inline void PID3::reset()
{
  ei_.setZero();
}

inline const Eigen::Vector3d& PID3::integralError() const
{
  return ei_;
}
}  // namespace ctrl
