#include <gazebo/gazebo.hh>

#include <dh_std_tools/algorithm.hpp>

#include "../../include/tobas_gazebo_plugins/fixed_wing_tools.hpp"

using namespace std;

namespace gazebo
{
ControlSurface::ControlSurface() : angle_(0.)
{
}

void ControlSurface::setAngle(double cmd_angle, double dt)
{
  assert(dt > 0.);

  if (!angle_limit.inRange(cmd_angle))
  {
    gzerr << "Commanded angle " << cmd_angle << " is out of range " << angle_limit << "." << endl;
    cmd_angle = angle_limit.clamp(cmd_angle);
  }

  // 速度制限
  double ideal_delta_angle = cmd_angle - angle_;
  double max_delta_angle = max_angle_rate * dt;
  double delta_angle = dh_std::clamp(ideal_delta_angle, -max_delta_angle, max_delta_angle);

  // 位置制限
  double cnd_angle = angle_ + delta_angle;
  angle_ = dh_std::clamp(cnd_angle, angle_limit.lower, angle_limit.upper);
}

const double& ControlSurface::getAngle() const
{
  return angle_;
}
}  // namespace gazebo
