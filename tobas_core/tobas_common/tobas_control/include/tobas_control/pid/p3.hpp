#pragma once

#include <eigen3/Eigen/Core>

namespace ctrl
{
class P3
{
public:
  Eigen::Vector3d kp = Eigen::Vector3d::Zero();

  explicit P3();

  inline Eigen::Vector3d update(const Eigen::Vector3d& cur_pos, const Eigen::Vector3d& tar_pos);
};

inline Eigen::Vector3d P3::update(const Eigen::Vector3d& cur_pos, const Eigen::Vector3d& tar_pos)
{
  return kp.cwiseProduct(tar_pos - cur_pos);
}
}  // namespace ctrl
