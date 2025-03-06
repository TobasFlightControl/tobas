#include <QVBoxLayout>

#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/controller/active_tilt_mr_pid.hpp"

namespace gui
{
namespace sa
{
ActiveTiltMultirotorPIDWidget::ActiveTiltMultirotorPIDWidget(
  RobotInfo& robot,
  const propulsion::PropulsionSystemWidget* propulsion_system,
  const fixed_wing::FixedWingWidget* fixed_wing)
  : robot_(robot), propulsion_system_(propulsion_system), fixed_wing_(fixed_wing)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  rows->addStretch();
}

const char* ActiveTiltMultirotorPIDWidget::name() const
{
  return "Active Tilt Multirotor PID";
}

const char* ActiveTiltMultirotorPIDWidget::description() const
{
  return "This is a PID controller for active-tilt multirotors.\n"
         "The airframe must satisfy the following conditions:\n"
         "  - All rotors have active tilt joint.\n"
         "  - Tilt axes are perpendicular to corresponding rotor axes.\n"
         "  - Propellers are generally located on corresponding rotor axes.";
}

QString ActiveTiltMultirotorPIDWidget::controllerPackage() const
{
  return "tobas_tiltrotor_pid";
}

QString ActiveTiltMultirotorPIDWidget::pluginName() const
{
  return "ControllerNode";
}

tobas::rc_command_t ActiveTiltMultirotorPIDWidget::acrobatModeCommand() const
{
  return tobas::rc_command_t::POS_VEL_ANGLE;
}

tobas::rc_command_t ActiveTiltMultirotorPIDWidget::stabilizeModeCommand() const
{
  return tobas::rc_command_t::ACCEL_ANGLE;
}

tobas::rc_command_t ActiveTiltMultirotorPIDWidget::loiterModeCommand() const
{
  return tobas::rc_command_t::ACCEL_RATE;
}

YAML::Node ActiveTiltMultirotorPIDWidget::staticParams() const
{
  return YAML::Node(YAML::NodeType::Map);
}

YAML::Node ActiveTiltMultirotorPIDWidget::dump() const
{
  return YAML::Node(YAML::NodeType::Map);
}

void ActiveTiltMultirotorPIDWidget::load(const YAML::Node&)
{
}

bool ActiveTiltMultirotorPIDWidget::isApplicable()
{
  const auto props = propulsion_system_->selected();

  // 固定翼を持たない
  if (fixed_wing_->hasFixedWing())
    return false;

  // プロペラの個数条件
  if (props->count() < kMinNumProp)
    return false;

  for (int i = 0; i < props->count(); ++i)
  {
    const auto prop = props->widget(i);
    const auto link_name = props->linkName(i);

    // 全てティルトロータ
    // TODO: アクティブティルトと固定モータの混合モデルも許容
    if (!prop->general()->isTiltRotor())
      return false;

    // TODO: ティルト軸とロータ軸が直行する (cf. tobas_drone_tools/tr_mixer_pinv)

    // TODO: プロペラリンクとティルト軸の距離が閾値以下
  }

  return true;
}

bool ActiveTiltMultirotorPIDWidget::isValid()
{
  return true;
}
}  // namespace sa
}  // namespace gui
