#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/controller/multirotor_pid.hpp"

namespace gui
{
namespace setup_assistant
{
MultirotorPIDWidget::MultirotorPIDWidget(
  RobotInfo& robot,
  const propulsion_system::PropulsionSystemWidget* propulsion_system,
  const fixed_wing::FixedWingWidget* fixed_wing)
  : robot_(robot), propulsion_system_(propulsion_system), fixed_wing_(fixed_wing)
{
}

const char* MultirotorPIDWidget::name() const
{
  return "Multirotor PID";
}

const char* MultirotorPIDWidget::description() const
{
  return "This controller for planar multirotors employs PID for both position and attitude control.";
}

const char* MultirotorPIDWidget::controllerPackage() const
{
  return "tobas_mr_pid";
}

const char* MultirotorPIDWidget::actionsPackage() const
{
  return "tobas_mr_actions";
}

tobas::rc_command_t MultirotorPIDWidget::stabilizeModeCommand() const
{
  return tobas::rc_command_t::POS_VEL_ACC_YAW;
}

tobas::rc_command_t MultirotorPIDWidget::acrobatModeCommand() const
{
  return tobas::rc_command_t::ROLL_PITCH_YAW_THROTTLE;
}

YAML::Node MultirotorPIDWidget::staticParams() const
{
  return YAML::Node();
}

YAML::Node MultirotorPIDWidget::dump() const
{
  return YAML::Node();
}

void MultirotorPIDWidget::load(const YAML::Node&)
{
}

bool MultirotorPIDWidget::isApplicable()
{
  // 固定翼を持たない
  if (fixed_wing_->hasFixedWing())
    return false;

  // プロペラの個数条件
  if (propulsion_system_->selected()->count() < kMinNumProp)
    return false;

  // Z軸正方向のプロペラのみ
  for (int i = 0; i < propulsion_system_->selected()->count(); ++i)
  {
    const auto link_name = propulsion_system_->selected()->linkName(i);
    if (!robot_.isJntAxisAlwaysCollinear(link_name.toStdString(), kdl::Vector::UnitZ()))
      return false;
  }

  return true;
}

bool MultirotorPIDWidget::isValid()
{
  // 両方の回転方向のプロペラをもつ
  if (!propulsion_system_->selected()->hasBothRotationalDirections())
  {
    qt::qErrorBox(
      this, "All rotors have the same rotation direction. "
            "Rotors that rotate in both clockwise (CW) and counterclockwise (CCW) are required.");
    return false;
  }

  return true;
}
}  // namespace setup_assistant
}  // namespace gui
