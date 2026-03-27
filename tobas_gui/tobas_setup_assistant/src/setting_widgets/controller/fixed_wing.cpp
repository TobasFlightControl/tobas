#include "tobas_setup_assistant/setting_tabs/controller/fixed_wing.hpp"

#include <QVBoxLayout>

namespace tobas
{
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

FrameType FixedWingWidget::frameType() const
{
  return FrameType::kFixedWing;
}

QString FixedWingWidget::controllerPackage() const
{
  return "tobas_fixed_wing_controller";
}

QString FixedWingWidget::pluginName() const
{
  return "tobas::fixed_wing::ControllerNode";
}

RcCommand FixedWingWidget::acrobatModeCommand() const
{
  return RcCommand::kSpeedRollDPitch;  // TODO
}

RcCommand FixedWingWidget::stabilizeModeCommand() const
{
  return RcCommand::kSpeedRollDPitch;  // TODO
}

RcCommand FixedWingWidget::loiterModeCommand() const
{
  return RcCommand::kSpeedRollDPitch;  // TODO
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
}  // namespace tobas
