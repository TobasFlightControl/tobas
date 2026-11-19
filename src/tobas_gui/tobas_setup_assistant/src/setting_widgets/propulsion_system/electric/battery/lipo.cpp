// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/battery/lipo.hpp"

#include <tobas_qt_tools/layouts/form_layout.hpp>

namespace tobas::gui::sa::propulsion::electric
{
namespace
{
constexpr char kNumCellsLabel[] = "Number of Cells";
constexpr char kCapacityLabel[] = "Battery Capacity";
constexpr char kContinuousDischargeRateLabel[] = "Continuous Discharge C-Rate";
constexpr char kResistanceLabel[] = "Internal Resistance";
}  // namespace

BatteryWidget_LiPo::BatteryWidget_LiPo()
{
  const auto form = new qt::FormLayout();
  setLayout(form);  // Do not pass `this` to the layout constructor.

  num_cells_ = new qt::SpinBox();
  num_cells_->setToolTip(
    "Number of cells connected in series. Used to calculate the pack voltage and total internal resistance.");
  num_cells_->setMinimum(1);
  num_cells_->setMaximum(100);
  form->addRow(kNumCellsLabel, num_cells_);
  form->labelForField(num_cells_)->setToolTip(num_cells_->toolTip());

  capacity_ = new qt::SpinBox();
  capacity_->setToolTip("Rated charge capacity of the battery pack in milliampere-hours (mAh).");
  capacity_->setMinimum(1);
  capacity_->setSuffix(" mAh");
  form->addRow(kCapacityLabel, capacity_);
  form->labelForField(capacity_)->setToolTip(capacity_->toolTip());

  c_cont_ = new qt::SpinBox();
  c_cont_->setToolTip(
    "Continuous discharge C-rate, expressed per hour. "
    "Maximum current in amperes equals capacity in ampere-hours multiplied by this value.");
  c_cont_->setMinimum(1);
  c_cont_->setSuffix(" /h");
  form->addRow(kContinuousDischargeRateLabel, c_cont_);
  form->labelForField(c_cont_)->setToolTip(c_cont_->toolTip());

  registance_ = new qt::SpinBox();
  registance_->setToolTip(
    "Internal resistance of one cell in milliohms. "
    "The pack resistance is this value multiplied by the number of series cells.");
  registance_->setMinimum(0);
  registance_->setSuffix(" mΩ");
  form->addRow(kResistanceLabel, registance_);
  form->labelForField(registance_)->setToolTip(registance_->toolTip());
}

const char* BatteryWidget_LiPo::name() const
{
  return "Lithium Polymer Battery (LiPo)";
}

void BatteryWidget_LiPo::setToDefaults()
{
  num_cells_->setValue(3);
  capacity_->setValue(5000);
  c_cont_->setValue(50);
  registance_->setValue(3);
}

bool BatteryWidget_LiPo::isValid()
{
  return true;
}

YAML::Node BatteryWidget_LiPo::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNumCellsLabel] = num_cells_->value();
  node[kCapacityLabel] = capacity_->value();
  node[kContinuousDischargeRateLabel] = c_cont_->value();
  node[kResistanceLabel] = registance_->value();

  return node;
}

void BatteryWidget_LiPo::load(const YAML::Node& _node)
{
  num_cells_->setValue(_node[kNumCellsLabel].as<int>());
  capacity_->setValue(_node[kCapacityLabel].as<int>());
  c_cont_->setValue(_node[kContinuousDischargeRateLabel].as<int>());
  registance_->setValue(_node[kResistanceLabel].as<int>());
}

double BatteryWidget_LiPo::nominalVoltage()
{
  constexpr double kNominalVoltagePerCell = 3.7;  // Nominal voltage per cell
  return num_cells_->value() * kNominalVoltagePerCell;
}

double BatteryWidget_LiPo::maxVoltage()
{
  constexpr double kMaxVoltagePerCell = 4.2;  // Maximum voltage per cell
  return num_cells_->value() * kMaxVoltagePerCell;
}

double BatteryWidget_LiPo::sagVoltage()
{
  constexpr double kSagVoltagePerCell = 3.4;  // Voltage where discharge characteristics change abruptly
  return num_cells_->value() * kSagVoltagePerCell;
}

double BatteryWidget_LiPo::maxCurrent()
{
  return capacity_->value() * c_cont_->value() * 1e-3;
}

double BatteryWidget_LiPo::capacity()
{
  return capacity_->value() * 3600 * 1e-3;
}

double BatteryWidget_LiPo::internalRegistance()
{
  return num_cells_->value() * registance_->value() * 1e-3;
}
}  // namespace tobas::gui::sa::propulsion::electric
