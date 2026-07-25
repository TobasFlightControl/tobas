// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/controller/custom.hpp"

#include <QVBoxLayout>
#include <magic_enum/magic_enum.hpp>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace ctrl
{
CustomFrameWidget::CustomFrameWidget()
{
  TOBAS_CHECK(command_map_.size() == magic_enum::enum_count<RcCommand>());

  acrobat_mode_ = new qt::ComboBox();
  stabilize_mode_ = new qt::ComboBox();
  loiter_mode_ = new qt::ComboBox();

  // Add command choices.
  for (const auto& [text, _] : command_map_) {
    acrobat_mode_->addItem(text);
    stabilize_mode_->addItem(text);
    loiter_mode_->addItem(text);
  }

  // Set default command.
  acrobat_mode_->setCurrentText(kRateThrottleLabel);
  stabilize_mode_->setCurrentText(kAccelYawLabel);
  loiter_mode_->setCurrentText(kPosVelAccYawLabel);

  // Layout
  const auto form = new qt::FormLayout();
  form->addRow(kAcrobatLabel, acrobat_mode_);
  form->addRow(kStabilizeLabel, stabilize_mode_);
  form->addRow(kLoiterLabel, loiter_mode_);

  const auto rows = new QVBoxLayout();
  rows->addWidget(new qt::Label("RC Command", cmn::kLabelPSize, QFont::Bold));
  rows->addLayout(form);
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

RcCommand CustomFrameWidget::acrobatModeCommand() const
{
  return command_map_.at(acrobat_mode_->currentText());
}

RcCommand CustomFrameWidget::stabilizeModeCommand() const
{
  return command_map_.at(stabilize_mode_->currentText());
}

RcCommand CustomFrameWidget::loiterModeCommand() const
{
  return command_map_.at(loiter_mode_->currentText());
}

YAML::Node CustomFrameWidget::staticParams() const
{
  return YAML::Node(YAML::NodeType::Map);
}

YAML::Node CustomFrameWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kAcrobatLabel] = acrobat_mode_->currentText();
  node[kStabilizeLabel] = stabilize_mode_->currentText();
  node[kLoiterLabel] = loiter_mode_->currentText();

  return node;
}

void CustomFrameWidget::load(const YAML::Node& node)
{
  acrobat_mode_->setCurrentText(node[kAcrobatLabel].as<QString>());
  stabilize_mode_->setCurrentText(node[kStabilizeLabel].as<QString>());
  loiter_mode_->setCurrentText(node[kLoiterLabel].as<QString>());
}

bool CustomFrameWidget::isValid()
{
  return true;
}
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
}  // namespace tobas
