#include "../include/tobas_rc_teleop/program_mode.hpp"

namespace tobas_rc_teleop
{
ProgramModeController::ProgramModeController()
{
}

void ProgramModeController::initialize(tobas::BaseNode*)
{
}

void ProgramModeController::reset(const tobas_msgs::Odometry&)
{
}

void ProgramModeController::update(const tobas_msgs::msg::RCInput&, const tobas_msgs::Odometry&)
{
}
}  // namespace tobas_rc_teleop
