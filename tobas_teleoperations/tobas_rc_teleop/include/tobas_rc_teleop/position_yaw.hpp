#pragma once

#include <tobas_msgs/PositionYaw.h>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PositionYawController : public BaseController
{
  using super = BaseController;

public:
  explicit PositionYawController(const tobas::Drone& drone);

  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(
    const tobas_msgs::RCInput& rcin,
    const tobas_msgs::Odometry& odom,
    const double& battery_voltage) override;

private:
  bool is_up_commanded_ = false;
  tobas_msgs::PositionYaw pos_yaw_;
  tobas_kdl::Vector vel_;
  ros::Time t_last_rcin_;

  // rosparams
  double max_hor_vel_;  // [m/s]
  double max_ver_vel_;  // [m/s]
  double max_yawrate_;  // [rad/s]

  // Publisher
  ros::Publisher pos_yaw_pub_;

  void getRosParams(ros::NodeHandle& pnh);
};
}  // namespace tobas_rc_teleop
