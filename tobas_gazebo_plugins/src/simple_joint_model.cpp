#include <gazebo/gazebo.hh>

#include "../include/tobas_gazebo_plugins/simple_joint_model.hpp"

#define POS_MARGIN 1e-2  // [rad]

using namespace std;

namespace gazebo
{
SimpleJointModel::SimpleJointModel(const tobas_std::Range<double>& pos_limit, const double& max_vel)
  : pos_limit_(pos_limit), max_vel_(max_vel)
{
  assert(pos_limit.isValid());
  assert(max_vel > 0.);

  cur_pos_ = 0.;
}

void SimpleJointModel::update(double tar_pos, double dt)
{
  assert(dt >= 0);

  if (!pos_limit_.inRange(tar_pos, POS_MARGIN))
  {
    gzerr << "The target position " << tar_pos << "[rad] is out of range " << pos_limit_ << "[rad]."
          << endl;
    tar_pos = pos_limit_.clamp(tar_pos);
  }

  // 速度制限
  const auto ideal_delta_angle = tar_pos - cur_pos_;
  const auto max_delta_angle = max_vel_ * dt;
  const auto delta_angle = clamp(ideal_delta_angle, -max_delta_angle, max_delta_angle);

  // 位置制限
  const auto cnd_angle = cur_pos_ + delta_angle;
  cur_pos_ = pos_limit_.clamp(cnd_angle);
}

const double& SimpleJointModel::currentPosition() const
{
  return cur_pos_;
}
}  // namespace gazebo
