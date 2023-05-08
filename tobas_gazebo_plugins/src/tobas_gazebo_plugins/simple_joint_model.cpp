#include <gazebo/gazebo.hh>

#include "../../include/tobas_gazebo_plugins/simple_joint_model.hpp"

using namespace std;

namespace gazebo
{
SimpleJointModel::SimpleJointModel(const dh_std::Range<double>& pos_limit, const double max_vel)
  : pos_limit_(pos_limit), max_vel_(max_vel)
{
  cur_pos_ = 0.;
}

void SimpleJointModel::setTargetPosition(double tar_pos, double dt)
{
  assert(dt > 0.);

  if (!pos_limit_.inRange(tar_pos))
  {
    gzerr << "Target position " << tar_pos << " is out of range " << pos_limit_ << "." << endl;
    tar_pos = pos_limit_.clamp(tar_pos);
  }

  // 速度制限
  double ideal_delta_angle = tar_pos - cur_pos_;
  double max_delta_angle = max_vel_ * dt;
  double delta_angle = dh_std::clamp(ideal_delta_angle, -max_delta_angle, max_delta_angle);

  // 位置制限
  double cnd_angle = cur_pos_ + delta_angle;
  cur_pos_ = dh_std::clamp(cnd_angle, pos_limit_.lower, pos_limit_.upper);
}

const double& SimpleJointModel::getCurrentPosition() const
{
  return cur_pos_;
}
}  // namespace gazebo
