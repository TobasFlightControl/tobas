// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/rc_input.hpp"

#include <tobas_constants/rc_input.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace
{
// Keep the existing serialized keys independent of the form labels.
constexpr char kNumSbusChannelsKey[] = "S.BUS Channels";
constexpr char kArmDurationKey[] = "Arm Duration";
constexpr char kDisarmDurationKey[] = "Disarm Duration";
}  // namespace

RcInputWidget::RcInputWidget()
{
  const auto form = new qt::FormLayout();
  addLayout(form);

  num_sbus_channels_ = new qt::SpinBox();
  num_sbus_channels_->setToolTip("Specifies the number of S.BUS channels actually used in manual control mode.");
  num_sbus_channels_->setMinimum(kMinSbusChannels);
  num_sbus_channels_->setMaximum(kMaxSbusChannels);
  num_sbus_channels_->setValue(kMinSbusChannels);
  form->addRow(kNumSbusChannelsKey, num_sbus_channels_);
  form->labelForField(num_sbus_channels_)->setToolTip(num_sbus_channels_->toolTip());

  arm_duration_ = new qt::DoubleSpinBox();
  arm_duration_->setToolTip(
    "Specifies the amount of time the stick must be held to arm the vehicle from the transmitter.");
  arm_duration_->setDecimals(1);
  arm_duration_->setMinimum(0.0);
  arm_duration_->setMaximum(10.0);
  arm_duration_->setValue(1.0);
  arm_duration_->setSuffix(" s");
  form->addRow(kArmDurationKey, arm_duration_);
  form->labelForField(arm_duration_)->setToolTip(arm_duration_->toolTip());

  disarm_duration_ = new qt::DoubleSpinBox();
  disarm_duration_->setToolTip(
    "Specifies the amount of time the stick must be held to disarm the vehicle from the transmitter.");
  disarm_duration_->setDecimals(1);
  disarm_duration_->setMinimum(0.0);
  disarm_duration_->setMaximum(10.0);
  disarm_duration_->setValue(1.0);
  disarm_duration_->setSuffix(" s");
  form->addRow(kDisarmDurationKey, disarm_duration_);
  form->labelForField(disarm_duration_)->setToolTip(disarm_duration_->toolTip());

  addStretch();
}

const char* RcInputWidget::name() const
{
  return "RC Input";
}

const char* RcInputWidget::title() const
{
  return "Configure Radio Control";
}

const char* RcInputWidget::description() const
{
  return "Configure the parameters for transmitter-based remote operation. "
         "Tobas supports S.BUS and S.BUS2 receiver protocols. "
         "Enter the appropriate values in each field.";
}

void RcInputWidget::updateInternalDataStructures()
{
}

bool RcInputWidget::isValid()
{
  return true;
}

YAML::Node RcInputWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNumSbusChannelsKey] = num_sbus_channels_->value();
  node[kArmDurationKey] = yaml::format(arm_duration_->value());
  node[kDisarmDurationKey] = yaml::format(disarm_duration_->value());

  return node;
}

void RcInputWidget::load(const YAML::Node& node)
{
  num_sbus_channels_->setValue(node[kNumSbusChannelsKey].as<int>());
  arm_duration_->setValue(node[kArmDurationKey].as<double>());
  disarm_duration_->setValue(node[kDisarmDurationKey].as<double>());
}

int RcInputWidget::numOfSbusChannels() const
{
  return num_sbus_channels_->value();
}

double RcInputWidget::armDuration() const
{
  return arm_duration_->value();
}

double RcInputWidget::disarmDuration() const
{
  return disarm_duration_->value();
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
