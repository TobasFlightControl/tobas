#pragma once

#include <eigen3/Eigen/Core>

namespace ctrl
{
class PID3
{
public:
  Eigen::Vector3d kp = Eigen::Vector3d::Zero();
  Eigen::Vector3d ki = Eigen::Vector3d::Zero();
  Eigen::Vector3d kd = Eigen::Vector3d::Zero();
  Eigen::Vector3d i_max = Eigen::Vector3d::Constant(INFINITY);

  explicit PID3();

  Eigen::Vector3d update(
    const Eigen::Vector3d& cur_pos,
    const Eigen::Vector3d& cur_vel,
    const Eigen::Vector3d& tar_pos,
    const Eigen::Vector3d& tar_vel,
    const double& dt);

  inline void reset();

  inline const Eigen::Vector3d& getIntegralError() const;

private:
  Eigen::Vector3d ei_ = Eigen::Vector3d::Zero();
};

inline void PID3::reset()
{
  ei_.setZero();
}

inline const Eigen::Vector3d& PID3::getIntegralError() const
{
  return ei_;
}
}  // namespace ctrl
