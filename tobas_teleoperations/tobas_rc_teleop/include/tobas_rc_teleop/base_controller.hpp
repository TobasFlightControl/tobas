#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/range.hpp>

#include <tobas_drone_core/drone.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/RCInput.h>

namespace tobas_rc_teleop
{
class BaseController
{
public:
  explicit BaseController(const tobas::Drone& drone);

  virtual void initialize() = 0;
  virtual void reset(const tobas_msgs::Odometry& odom) = 0;
  virtual void
  update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, const double& battery_voltage) = 0;

protected:
  const tobas::Drone& drone_;
  const tobas_std::Range<double> dead_zone_;

  /* RCInputの値を範囲[a, b]に投影する． */
  inline double remap(const double& x, const double& a, const double& b);

  /* RCInputの値がdead_zoneに入っていたら0，入っていなければ[a, b]に投影する． */
  inline double remapDead(const double& x, const double& a, const double& b);
};

inline double BaseController::remap(const double& x, const double& a, const double& b)
{
  return math::remap(x, tobas::kRCInputMin, tobas::kRCInputMax, a, b);
}

inline double BaseController::remapDead(const double& x, const double& a, const double& b)
{
  return dead_zone_.inRange(x) ? 0. : remap(x, a, b);
}
}  // namespace tobas_rc_teleop
