#pragma once

#include <tobas_std_tools/range.hpp>

namespace gazebo
{
/* 位置と最大速度の制約を含む単純な関節モデル． */
class SimpleJointModel
{
public:
  explicit SimpleJointModel(double min_pos, double max_pos, double max_vel);

  void update(double tar_pos, double dt);
  double currentPosition() const;

private:
  const tobas_std::Range<double> pos_limit_;
  const double max_vel_;

  double cur_pos_ = 0.;
};
}  // namespace gazebo
