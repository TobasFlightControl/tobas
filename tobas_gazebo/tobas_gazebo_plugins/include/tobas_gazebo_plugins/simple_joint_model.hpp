#pragma once

#include <tobas_std_tools/range.hpp>

namespace gazebo
{
/* 位置と最大速度の制約を含む単純な関節モデル． */
class SimpleJointModel
{
public:
  tobas_std::Range<double> pos_limit;
  double max_vel;

  explicit SimpleJointModel(double _min_pos, double _max_pos, double _max_vel);
  explicit SimpleJointModel();

  double getCurrentPosition() const;
  void setTargetPosition(double tar_pos);
  void step(double dt);

private:
  double cur_pos_ = 0.;
  double tar_pos_ = 0.;
};
}  // namespace gazebo
