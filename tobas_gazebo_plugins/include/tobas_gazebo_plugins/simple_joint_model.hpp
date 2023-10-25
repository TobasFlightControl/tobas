#pragma once

#include <dh_std_tools/range.hpp>

namespace gazebo
{
/* 位置と最大速度の制約を含む単純な関節モデル． */
class SimpleJointModel
{
public:
  explicit SimpleJointModel(const dh_std::Range<double>& pos_limit, const double& max_vel);

  void update(double tar_pos, double dt);
  const double& currentPosition() const;

private:
  const dh_std::Range<double> pos_limit_;
  const double max_vel_;

  double cur_pos_;
};
}  // namespace gazebo
