#include "tobas_setup_assistant/setting_tabs/controller/fixed_wing_lqr.hpp"

#include <QVBoxLayout>

namespace gui
{
namespace sa
{
FixedWingLQRWidget::FixedWingLQRWidget(RobotInfo& robot) : robot_(robot)
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

tobas::rc_command_t FixedWingLQRWidget::acrobatModeCommand() const
{
  return tobas::rc_command_t::SPEED_ROLL_DPITCH;  // TODO
}

tobas::rc_command_t FixedWingLQRWidget::stabilizeModeCommand() const
{
  return tobas::rc_command_t::SPEED_ROLL_DPITCH;  // TODO
}

tobas::rc_command_t FixedWingLQRWidget::loiterModeCommand() const
{
  return tobas::rc_command_t::SPEED_ROLL_DPITCH;  // TODO
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
  // 固定翼の操舵面の個数条件
  if (robot_.uadf().control_surfaces.size() < kMinNumCS) {
    return false;
  }

  // プロペラの個数条件
  if (robot_.uadf().thrusts.size() < kMinNumProp) {
    return false;
  }

  // X軸正方向のプロペラのみ
  for (const auto& [joint_name, _] : robot_.uadf().thrusts) {
    const auto& link_name = robot_.linkName(joint_name);
    if (!robot_.isJntAxisAlwaysCollinear(link_name, kdl::Vector::UnitX())) {
      return false;
    }
  }

  return true;
}

bool FixedWingLQRWidget::isValid()
{
  // TODO: 制御面の数や符号などに関する条件
  return true;
}
}  // namespace sa
}  // namespace gui
