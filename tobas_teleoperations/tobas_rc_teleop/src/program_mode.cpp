#include "../include/tobas_rc_teleop/program_mode.hpp"

namespace tobas_rc_teleop
{
ProgramModeController::ProgramModeController(const tobas::Drone& drone) : BaseController(drone)
{
}

void ProgramModeController::initialize(ros::NodeHandle&, ros::NodeHandle&)
{
}

void ProgramModeController::reset(const tobas_msgs::Odometry&)
{
}

void ProgramModeController::update(const tobas_msgs::RCInput&, const tobas_msgs::Odometry&, const double&)
{
}
}  // namespace tobas_rc_teleop
