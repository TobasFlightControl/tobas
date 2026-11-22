// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/rc_input.hpp"

#include <QFormLayout>

#include <tobas_constants/rc_input.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas::gui::sa
{
namespace
{
constexpr char kNumSbusChannelsLabel[] = "S.BUS Channels";
constexpr char kArmDurationLabel[] = "Arm Duration";
constexpr char kDisarmDurationLabel[] = "Disarm Duration";
}  // namespace

RcInputWidget::RcInputWidget()
{
  num_sbus_channels_ = new qt::SpinBox();
  num_sbus_channels_->setToolTip("Number of S.BUS channels used for transmitter input.");
  num_sbus_channels_->setMinimum(kMinSbusChannels);
  num_sbus_channels_->setMaximum(kMaxSbusChannels);

  arm_duration_ = new qt::DoubleSpinBox();
  arm_duration_->setToolTip(
    "Time in seconds that the transmitter sticks must remain in the arming position before the vehicle arms.");
  arm_duration_->setDecimals(1);
  arm_duration_->setMinimum(0.0);
  arm_duration_->setMaximum(10.0);
  arm_duration_->setSuffix(" s");

  disarm_duration_ = new qt::DoubleSpinBox();
  disarm_duration_->setToolTip(
    "Time in seconds that the transmitter sticks must remain in the disarming position before the vehicle disarms.");
  disarm_duration_->setDecimals(1);
  disarm_duration_->setMinimum(0.0);
  disarm_duration_->setMaximum(10.0);
  disarm_duration_->setSuffix(" s");

  const auto form = new QFormLayout(this);
  form->addRow(kNumSbusChannelsLabel, num_sbus_channels_);
  form->labelForField(num_sbus_channels_)->setToolTip(num_sbus_channels_->toolTip());
  form->addRow(kArmDurationLabel, arm_duration_);
  form->labelForField(arm_duration_)->setToolTip(arm_duration_->toolTip());
  form->addRow(kDisarmDurationLabel, disarm_duration_);
  form->labelForField(disarm_duration_)->setToolTip(disarm_duration_->toolTip());
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
  return "Configure the S.BUS channel count and the transmitter stick hold times used to arm and disarm the vehicle.";
}

void RcInputWidget::updateInternalDataStructures()
{
}

void RcInputWidget::setToDefaults()
{
  num_sbus_channels_->setValue(kMinSbusChannels);
  arm_duration_->setValue(1.0);
  disarm_duration_->setValue(1.0);
}

bool RcInputWidget::isValid()
{
  return true;
}

YAML::Node RcInputWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNumSbusChannelsLabel] = num_sbus_channels_->value();
  node[kArmDurationLabel] = yaml::format(arm_duration_->value());
  node[kDisarmDurationLabel] = yaml::format(disarm_duration_->value());

  return node;
}

void RcInputWidget::load(const YAML::Node& _node)
{
  num_sbus_channels_->setValue(_node[kNumSbusChannelsLabel].as<int>());
  arm_duration_->setValue(_node[kArmDurationLabel].as<double>());
  disarm_duration_->setValue(_node[kDisarmDurationLabel].as<double>());
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
}  // namespace tobas::gui::sa
