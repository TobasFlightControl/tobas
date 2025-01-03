#include <QVBoxLayout>

#include "tobas_setup_assistant/setting_tabs/controller/fixed_wing_lqr.hpp"

namespace gui
{
namespace setup_assistant
{
FixedWingLQRWidget::FixedWingLQRWidget(
  RobotInfo& robot,
  const propulsion_system::PropulsionSystemWidget* propulsion_system,
  const fixed_wing::FixedWingWidget* fixed_wing)
  : robot_(robot), propulsion_system_(propulsion_system), fixed_wing_(fixed_wing)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  rows->addStretch();
}

const char* FixedWingLQRWidget::name() const
{
  return "Fixed Wing LQR";
}

const char* FixedWingLQRWidget::description() const
{
  return "Control the fixed-wing aircraft using LQR (Linear Quadratic Regulator). "
         "While this method is computationally light, it does not consider hard constraints, "
         "which may lead to the issuance of commands outside the permissible range.";
}

QString FixedWingLQRWidget::controllerPackage() const
{
  return "tobas_fixed_wing_lqd";
}

QString FixedWingLQRWidget::pluginName() const
{
  return "ControllerNode";
}

tobas::rc_command_t FixedWingLQRWidget::stabilizeModeCommand() const
{
  return tobas::rc_command_t::SPEED_ROLL_DPITCH;
}

tobas::rc_command_t FixedWingLQRWidget::acrobatModeCommand() const
{
  return tobas::rc_command_t::SPEED_ROLL_DPITCH;  // TODO: S
}

YAML::Node FixedWingLQRWidget::staticParams() const
{
  return YAML::Node(YAML::NodeType::Map);
}

YAML::Node FixedWingLQRWidget::dump() const
{
  return YAML::Node(YAML::NodeType::Map);
}

void FixedWingLQRWidget::load(const YAML::Node&)
{
}

bool FixedWingLQRWidget::isApplicable()
{
  const auto props = propulsion_system_->selected();

  // 固定翼を持つ
  if (!fixed_wing_->hasFixedWing())
    return false;

  // 制御面の個数条件
  if (fixed_wing_->controlSurfaces()->count() < kMinNumCS)
    return false;

  // プロペラの個数条件
  if (props->count() < kMinNumProp)
    return false;

  // X軸正方向のプロペラのみ
  for (int i = 0; i < props->count(); ++i)
  {
    const auto link_name = props->linkName(i);
    if (!robot_.isJntAxisAlwaysCollinear(link_name.toStdString(), kdl::Vector::UnitX()))
      return false;
  }

  return true;
}

bool FixedWingLQRWidget::isValid()
{
  // TODO: 制御面の数や符号などに関する条件
  return true;
}
}  // namespace setup_assistant
}  // namespace gui
