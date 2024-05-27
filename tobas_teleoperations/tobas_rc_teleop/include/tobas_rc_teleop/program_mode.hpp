#pragma once

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class ProgramModeController : public BaseController
{
public:
  explicit ProgramModeController(const tobas::Drone& drone);

  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void
  update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, const double& battery_voltage) override;
};
}  // namespace tobas_rc_teleop
