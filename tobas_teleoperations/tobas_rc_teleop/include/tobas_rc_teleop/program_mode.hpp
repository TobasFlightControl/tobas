#pragma once

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class ProgramModeController : public BaseController
{
public:
  explicit ProgramModeController();

  void initialize(tobas::BaseNode* node) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void
  update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry& odom) override;
};
}  // namespace tobas_rc_teleop
