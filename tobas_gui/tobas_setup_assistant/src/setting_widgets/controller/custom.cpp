#include "tobas_setup_assistant/setting_tabs/controller/custom.hpp"

#include <QVBoxLayout>
#include <magic_enum/magic_enum.hpp>

#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace gui
{
namespace sa
{
namespace ctrl
{
CustomFrameWidget::CustomFrameWidget()
{
  TOBAS_CHECK(command_map_.size() == magic_enum::enum_count<tobas::RcCommand>());

  acrobat_mode_ = new ParamGetterWidget_ComboBox("Acrobat Mode", "");
  stabilize_mode_ = new ParamGetterWidget_ComboBox("Stabilize Mode", "");
  loiter_mode_ = new ParamGetterWidget_ComboBox("Loiter Mode", "");

  // Add command choices
  for (const auto& [text, _] : command_map_) {
    acrobat_mode_->addChoice(text);
    stabilize_mode_->addChoice(text);
    loiter_mode_->addChoice(text);
  }

  // Set default command
  acrobat_mode_->setValue(kRateThrottleLabel);
  stabilize_mode_->setValue(kAccelYawLabel);
  loiter_mode_->setValue(kPosVelYawLabel);

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(acrobat_mode_);
  rows->addWidget(stabilize_mode_);
  rows->addWidget(loiter_mode_);
  rows->addStretch();
  setLayout(rows);
}

FrameType CustomFrameWidget::frameType() const
{
  return FrameType::kUndefined;
}

QString CustomFrameWidget::controllerPackage() const
{
  return "tobas_dummy_pkg";
}

QString CustomFrameWidget::pluginName() const
{
  return "tobas::DummyNode";
}

tobas::RcCommand CustomFrameWidget::acrobatModeCommand() const
{
  return command_map_.at(acrobat_mode_->getValue());
}

tobas::RcCommand CustomFrameWidget::stabilizeModeCommand() const
{
  return command_map_.at(stabilize_mode_->getValue());
}

tobas::RcCommand CustomFrameWidget::loiterModeCommand() const
{
  return command_map_.at(loiter_mode_->getValue());
}

YAML::Node CustomFrameWidget::staticParams() const
{
  return YAML::Node(YAML::NodeType::Map);
}

YAML::Node CustomFrameWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[acrobat_mode_->name()] = acrobat_mode_->getValue();
  node[stabilize_mode_->name()] = stabilize_mode_->getValue();
  node[loiter_mode_->name()] = loiter_mode_->getValue();

  return node;
}

void CustomFrameWidget::load(const YAML::Node& node)
{
  acrobat_mode_->setValue(node[acrobat_mode_->name()].as<QString>());
  stabilize_mode_->setValue(node[stabilize_mode_->name()].as<QString>());
  loiter_mode_->setValue(node[loiter_mode_->name()].as<QString>());
}

bool CustomFrameWidget::isValid()
{
  return true;
}
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
