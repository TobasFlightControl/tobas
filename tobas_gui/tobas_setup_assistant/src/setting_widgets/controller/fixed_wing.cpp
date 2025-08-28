#include "tobas_setup_assistant/setting_tabs/controller/fixed_wing.hpp"

#include <QVBoxLayout>

namespace gui
{
namespace sa
{
namespace ctrl
{
FixedWingWidget::FixedWingWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  rows->addStretch();
}

const char* FixedWingWidget::name() const
{
  return "Fixed Wing LQR";
}

QString FixedWingWidget::controllerPackage() const
{
  return "tobas_fixed_wing_controller";
}

QString FixedWingWidget::pluginName() const
{
  return "ControllerNode";
}

tobas::RcCommand FixedWingWidget::acrobatModeCommand() const
{
  return tobas::RcCommand::kSpeedRollDPitch;  // TODO
}

tobas::RcCommand FixedWingWidget::stabilizeModeCommand() const
{
  return tobas::RcCommand::kSpeedRollDPitch;  // TODO
}

tobas::RcCommand FixedWingWidget::loiterModeCommand() const
{
  return tobas::RcCommand::kSpeedRollDPitch;  // TODO
}

YAML::Node FixedWingWidget::staticParams() const
{
  return YAML::Node(YAML::NodeType::Map);
}

YAML::Node FixedWingWidget::dump() const
{
  return YAML::Node(YAML::NodeType::Map);
}

void FixedWingWidget::load(const YAML::Node&)
{
}

bool FixedWingWidget::isValid()
{
  // TODO: 制御面の数や符号などに関する条件
  return true;
}
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
