#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/controller/non_planar_pid.hpp"

namespace gui
{
namespace setup_assistant
{
NonPlanarPIDWidget::NonPlanarPIDWidget(
  RobotInfo& robot,
  const propulsion_system::PropulsionSystemWidget* propulsion_system,
  const fixed_wing::FixedWingWidget* fixed_wing)
  : robot_(robot), propulsion_system_(propulsion_system), fixed_wing_(fixed_wing)
{
}

const char* NonPlanarPIDWidget::name() const
{
  return "Non-Planar Multirotor PID";
}

const char* NonPlanarPIDWidget::description() const
{
  return "This is a PID controller for non-planar multirotors.";
}

const char* NonPlanarPIDWidget::controllerPackage() const
{
  return "tobas_np_pid";
}

const char* NonPlanarPIDWidget::actionsPackage() const
{
  return "tobas_dummy_pkg";  // TODO
}

tobas::rc_command_t NonPlanarPIDWidget::stabilizeModeCommand() const
{
  return tobas::rc_command_t::POSE_TWIST_ACCEL;
}

tobas::rc_command_t NonPlanarPIDWidget::acrobatModeCommand() const
{
  return tobas::rc_command_t::POSE_TWIST_ACCEL;  // TODO
}

YAML::Node NonPlanarPIDWidget::staticParams() const
{
  return YAML::Node();
}

YAML::Node NonPlanarPIDWidget::dump() const
{
  return YAML::Node();
}

void NonPlanarPIDWidget::load(const YAML::Node&)
{
}

bool NonPlanarPIDWidget::isApplicable()
{
  // 固定翼を持たない
  if (fixed_wing_->hasFixedWing())
    return false;

  // プロペラの個数条件
  if (propulsion_system_->selected()->count() < kMinNumProp)
    return false;

  // 少なくとも1つのプロペラが鉛直上方向以外を向いている
  bool tilted_rotor_found = false;
  for (int i = 0; i < propulsion_system_->selected()->count(); ++i)
  {
    const auto link_name = propulsion_system_->selected()->linkName(i);
    if (!robot_.isJntAxisAlwaysCollinear(link_name.toStdString(), kdl::Vector::UnitZ()))
    {
      tilted_rotor_found = true;
      break;
    }
  }
  if (!tilted_rotor_found)
    return false;

  return true;
}

bool NonPlanarPIDWidget::isValid()
{
  // 全てのプロペラの回転方向が同じ場合は警告
  if (!propulsion_system_->selected()->hasBothRotationalDirections())
  {
    if (!qt::yesOrNo(this, "All rotors have the same rotation direction. Is that OK?", qt::QMessageLevel::WARN))
      return false;
  }

  return true;
}
}  // namespace setup_assistant
}  // namespace gui
