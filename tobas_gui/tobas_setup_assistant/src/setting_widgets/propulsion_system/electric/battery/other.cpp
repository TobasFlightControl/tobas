// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/battery/other.hpp"

#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
namespace
{
// Keep the existing serialized keys independent of the form labels.
constexpr char kNominalVoltageKey[] = "Nominal Voltage";
constexpr char kMaxVoltageKey[] = "Maximum Voltage";
constexpr char kSagVoltageKey[] = "Voltage Threshold";
constexpr char kMaxCurrentKey[] = "Maximum Current";
constexpr char kCapacityKey[] = "Current Capacity";
constexpr char kRegistanceKey[] = "Internal Registance";
}  // namespace

BatteryWidget_Other::BatteryWidget_Other()
{
  const auto form = new qt::FormLayout();
  setLayout(form);

  nominal_voltage_ = new qt::DoubleSpinBox();
  nominal_voltage_->setToolTip("Nominal voltage of the battery.");
  nominal_voltage_->setDecimals(1);
  nominal_voltage_->setMinimum(0.1);
  nominal_voltage_->setValue(11.1);
  nominal_voltage_->setSuffix(" V");
  form->addRow(kNominalVoltageKey, nominal_voltage_);
  form->labelForField(nominal_voltage_)->setToolTip(nominal_voltage_->toolTip());

  max_voltage_ = new qt::DoubleSpinBox();
  max_voltage_->setToolTip("Maximum voltage of the battery.");
  max_voltage_->setDecimals(1);
  max_voltage_->setMinimum(0.1);
  max_voltage_->setValue(12.6);
  max_voltage_->setSuffix(" V");
  form->addRow(kMaxVoltageKey, max_voltage_);
  form->labelForField(max_voltage_)->setToolTip(max_voltage_->toolTip());

  sag_voltage_ = new qt::DoubleSpinBox();
  sag_voltage_->setToolTip("Voltage at which the discharge characteristics change abruptly.");
  sag_voltage_->setDecimals(1);
  sag_voltage_->setMinimum(0.1);
  sag_voltage_->setValue(10.2);
  sag_voltage_->setSuffix(" V");
  form->addRow(kSagVoltageKey, sag_voltage_);
  form->labelForField(sag_voltage_)->setToolTip(sag_voltage_->toolTip());

  max_current_ = new qt::SpinBox();
  max_current_->setToolTip("Maximum current of the battery.");
  max_current_->setMinimum(1);
  max_current_->setValue(250);
  max_current_->setSuffix(" A");
  form->addRow(kMaxCurrentKey, max_current_);
  form->labelForField(max_current_)->setToolTip(max_current_->toolTip());

  capacity_ = new qt::SpinBox();
  capacity_->setToolTip("The amount of electric charge that can be drawn from the battery.");
  capacity_->setMinimum(1);
  capacity_->setValue(5000);
  capacity_->setSuffix(" mAh");
  form->addRow(kCapacityKey, capacity_);
  form->labelForField(capacity_)->setToolTip(capacity_->toolTip());

  registance_ = new qt::SpinBox();
  registance_->setToolTip("Internal resistance value per cell.");
  registance_->setMinimum(0);
  registance_->setValue(9);
  registance_->setSuffix(" mΩ");
  form->addRow(kRegistanceKey, registance_);
  form->labelForField(registance_)->setToolTip(registance_->toolTip());
}

const char* BatteryWidget_Other::name() const
{
  return "Other Battery";
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

  node[kNominalVoltageKey] = yaml::format(nominal_voltage_->value());
  node[kMaxVoltageKey] = yaml::format(max_voltage_->value());
  node[kSagVoltageKey] = yaml::format(sag_voltage_->value());
  node[kMaxCurrentKey] = max_current_->value();
  node[kCapacityKey] = capacity_->value();
  node[kRegistanceKey] = registance_->value();

  return node;
}

void BatteryWidget_Other::load(const YAML::Node& node)
{
  nominal_voltage_->setValue(node[kNominalVoltageKey].as<double>());
  max_voltage_->setValue(node[kMaxVoltageKey].as<double>());
  sag_voltage_->setValue(node[kSagVoltageKey].as<double>());
  max_current_->setValue(node[kMaxCurrentKey].as<double>());
  capacity_->setValue(node[kCapacityKey].as<int>());
  registance_->setValue(node[kRegistanceKey].as<int>());
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
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
