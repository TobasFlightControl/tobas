#pragma once

#include <dh_std_tools/range.hpp>

#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/RCInput.h>
#include <tobas_msgs/VelocityYaw.h>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class VelocityYawController : public BaseController
{
  static constexpr double kDefaultMaxHorizontalVelocity = 3.;  // [m/s]
  static constexpr double kDefaultMaxVerticalVelocity = 3.;    // [m/s]
  static constexpr double kDefaultMaxYawrate = M_PI;           // [rad/s]

  using super = BaseController;

public:
  explicit VelocityYawController();

  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) override;

  void reset(const tobas_msgs::PoseTwist& pt);
  void update(const tobas_msgs::RCInput& rcin, const dh_std::Range<double>& dead_zone);

private:
  tobas_msgs::VelocityYaw vel_yaw_;
  ros::Time t_last_rcin_;

  // rosparams
  double max_hor_vel_;  // [m/s]
  double max_ver_vel_;  // [m/s]
  double max_yawrate_;  // [rad/s]

  // Publisher
  ros::Publisher vel_yaw_pub_;

  void getRosParams(ros::NodeHandle& pnh);
  void registerPublishers(ros::NodeHandle& nh);
};
}  // namespace tobas_rc_teleop
