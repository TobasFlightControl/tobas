#include "../include/tobas_gazebo_plugins/simple_joint_model.hpp"

#define POS_MARGIN 1e-2  // [rad]

using namespace std;

namespace gazebo
{
SimpleJointModel::SimpleJointModel(double min_pos, double max_pos, double max_vel)
  : pos_limit_(min_pos, max_pos), max_vel_(max_vel)
{
  assert(pos_limit_.isValid());
  assert(max_vel_ > 0.);
}

void SimpleJointModel::update(double tar_pos, double dt)
{
  assert(dt >= 0);

  tar_pos = pos_limit_.clamp(tar_pos);

  // 速度制限
  const auto ideal_delta_angle = tar_pos - cur_pos_;
  const auto max_delta_angle = max_vel_ * dt;
  const auto delta_angle = clamp(ideal_delta_angle, -max_delta_angle, max_delta_angle);

  // 位置制限
  const auto cnd_angle = cur_pos_ + delta_angle;
  cur_pos_ = pos_limit_.clamp(cnd_angle);
}

double SimpleJointModel::currentPosition() const
{
  return cur_pos_;
}
}  // namespace gazebo
