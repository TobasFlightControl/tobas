// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/rc_input.hpp"

#include <tobas_constants/rc_input.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
RcInputWidget::RcInputWidget()
{
  num_sbus_channels_ = new ParamGetterWidget_SpinBox(
    "S.BUS Channels", "Specifies the number of S.BUS channels actually used in manual control mode.");
  num_sbus_channels_->setMinimum(kMinSbusChannels);
  num_sbus_channels_->setMaximum(kMaxSbusChannels);
  num_sbus_channels_->setValue(kMinSbusChannels);
  addWidget(num_sbus_channels_);

  arm_duration_ = new ParamGetterWidget_DoubleSpinBox(
    "Arm Duration", "Specifies the amount of time the stick must be held to arm the vehicle from the transmitter.");
  arm_duration_->setDecimals(1);
  arm_duration_->setMinimum(0.);
  arm_duration_->setMaximum(10.);
  arm_duration_->setValue(1.);
  arm_duration_->setSuffix(" s");
  addWidget(arm_duration_);

  disarm_duration_ = new ParamGetterWidget_DoubleSpinBox(
    "Disarm Duration",
    "Specifies the amount of time the stick must be held to disarm the vehicle from the transmitter.");
  disarm_duration_->setDecimals(1);
  disarm_duration_->setMinimum(0.);
  disarm_duration_->setMaximum(10.);
  disarm_duration_->setValue(1.);
  disarm_duration_->setSuffix(" s");
  addWidget(disarm_duration_);

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

  node[num_sbus_channels_->name()] = num_sbus_channels_->getValue();
  node[arm_duration_->name()] = yaml::format(arm_duration_->getValue());
  node[disarm_duration_->name()] = yaml::format(disarm_duration_->getValue());

  return node;
}

void RcInputWidget::load(const YAML::Node& node)
{
  num_sbus_channels_->setValue(node[num_sbus_channels_->name()].as<int>());
  arm_duration_->setValue(node[arm_duration_->name()].as<double>());
  disarm_duration_->setValue(node[disarm_duration_->name()].as<double>());
}

int RcInputWidget::numOfSbusChannels() const
{
  return num_sbus_channels_->getValue();
}

double RcInputWidget::armDuration() const
{
  return arm_duration_->getValue();
}

double RcInputWidget::disarmDuration() const
{
  return disarm_duration_->getValue();
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
