// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/battery/other.hpp"

#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas::gui::sa::propulsion::electric
{
namespace
{
constexpr char kNominalVoltageLabel[] = "Nominal Voltage";
constexpr char kMaxVoltageLabel[] = "Maximum Voltage";
constexpr char kSagVoltageLabel[] = "Voltage Threshold";
constexpr char kMaxCurrentLabel[] = "Maximum Current";
constexpr char kCapacityLabel[] = "Battery Capacity";
constexpr char kResistanceLabel[] = "Internal Resistance";
}  // namespace

BatteryWidget_Other::BatteryWidget_Other()
{
  const auto form = new qt::FormLayout();
  setLayout(form);  // Do not pass `this` to the layout constructor.

  nominal_voltage_ = new qt::DoubleSpinBox();
  nominal_voltage_->setToolTip("Nominal voltage of the complete battery pack in volts.");
  nominal_voltage_->setDecimals(1);
  nominal_voltage_->setMinimum(0.1);
  nominal_voltage_->setSuffix(" V");
  form->addRow(kNominalVoltageLabel, nominal_voltage_);
  form->labelForField(nominal_voltage_)->setToolTip(nominal_voltage_->toolTip());

  max_voltage_ = new qt::DoubleSpinBox();
  max_voltage_->setToolTip(
    "Voltage of the complete battery pack at full charge, in volts. Must exceed the voltage threshold.");
  max_voltage_->setDecimals(1);
  max_voltage_->setMinimum(0.1);
  max_voltage_->setSuffix(" V");
  form->addRow(kMaxVoltageLabel, max_voltage_);
  form->labelForField(max_voltage_)->setToolTip(max_voltage_->toolTip());

  sag_voltage_ = new qt::DoubleSpinBox();
  sag_voltage_->setToolTip(
    "Pack voltage in volts at the knee of the discharge curve, "
    "where the battery model changes its discharge characteristics.");
  sag_voltage_->setDecimals(1);
  sag_voltage_->setMinimum(0.1);
  sag_voltage_->setSuffix(" V");
  form->addRow(kSagVoltageLabel, sag_voltage_);
  form->labelForField(sag_voltage_)->setToolTip(sag_voltage_->toolTip());

  max_current_ = new qt::SpinBox();
  max_current_->setToolTip("Maximum continuous discharge current of the complete battery pack in amperes.");
  max_current_->setMinimum(1);
  max_current_->setSuffix(" A");
  form->addRow(kMaxCurrentLabel, max_current_);
  form->labelForField(max_current_)->setToolTip(max_current_->toolTip());

  capacity_ = new qt::SpinBox();
  capacity_->setToolTip("Rated charge capacity of the battery pack in milliampere-hours (mAh).");
  capacity_->setMinimum(1);
  capacity_->setSuffix(" mAh");
  form->addRow(kCapacityLabel, capacity_);
  form->labelForField(capacity_)->setToolTip(capacity_->toolTip());

  registance_ = new qt::SpinBox();
  registance_->setToolTip(
    "Total internal resistance of the complete battery pack in milliohms. Enter the pack value, not a per-cell value.");
  registance_->setMinimum(0);
  registance_->setSuffix(" mΩ");
  form->addRow(kResistanceLabel, registance_);
  form->labelForField(registance_)->setToolTip(registance_->toolTip());
}

const char* BatteryWidget_Other::name() const
{
  return "Other Battery";
}

void BatteryWidget_Other::setToDefaults()
{
  nominal_voltage_->setValue(11.1);
  max_voltage_->setValue(12.6);
  sag_voltage_->setValue(10.2);
  max_current_->setValue(250);
  capacity_->setValue(5000);
  registance_->setValue(9);
}

bool BatteryWidget_Other::isValid()
{
  if (max_voltage_->value() <= sag_voltage_->value()) {
    qt::qWarnBox(this, "Maximum voltage must be greater than voltage threshold.");
    return false;
  }

  return true;
}

YAML::Node BatteryWidget_Other::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNominalVoltageLabel] = yaml::format(nominal_voltage_->value());
  node[kMaxVoltageLabel] = yaml::format(max_voltage_->value());
  node[kSagVoltageLabel] = yaml::format(sag_voltage_->value());
  node[kMaxCurrentLabel] = max_current_->value();
  node[kCapacityLabel] = capacity_->value();
  node[kResistanceLabel] = registance_->value();

  return node;
}

void BatteryWidget_Other::load(const YAML::Node& _node)
{
  nominal_voltage_->setValue(_node[kNominalVoltageLabel].as<double>());
  max_voltage_->setValue(_node[kMaxVoltageLabel].as<double>());
  sag_voltage_->setValue(_node[kSagVoltageLabel].as<double>());
  max_current_->setValue(_node[kMaxCurrentLabel].as<double>());
  capacity_->setValue(_node[kCapacityLabel].as<int>());
  registance_->setValue(_node[kResistanceLabel].as<int>());
}

double BatteryWidget_Other::nominalVoltage()
{
  return nominal_voltage_->value();
}

double BatteryWidget_Other::maxVoltage()
{
  return max_voltage_->value();
}

double BatteryWidget_Other::sagVoltage()
{
  return sag_voltage_->value();
}

double BatteryWidget_Other::maxCurrent()
{
  return static_cast<double>(max_current_->value());
}

double BatteryWidget_Other::capacity()
{
  return static_cast<double>(capacity_->value() * 3600) * 1e-3;
}

double BatteryWidget_Other::internalRegistance()
{
  return static_cast<double>(registance_->value()) * 1e-3;
}
}  // namespace tobas::gui::sa::propulsion::electric
