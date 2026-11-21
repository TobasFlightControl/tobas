// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/battery/lipo.hpp"

#include <tobas_qt_tools/layouts/form_layout.hpp>

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
constexpr char kNumCellsKey[] = "Number of Cells";
constexpr char kCapacityKey[] = "Current Capacity";
constexpr char kCContKey[] = "Continuous Discharge Current Rate";
constexpr char kRegistanceKey[] = "Internal Registance";
}  // namespace

BatteryWidget_LiPo::BatteryWidget_LiPo()
{
  const auto form = new qt::FormLayout();
  setLayout(form);

  num_cells_ = new qt::SpinBox();
  num_cells_->setToolTip("The number of cells in the battery.");
  num_cells_->setMinimum(1);
  num_cells_->setMaximum(100);
  num_cells_->setValue(3);
  form->addRow(kNumCellsKey, num_cells_);
  form->labelForField(num_cells_)->setToolTip(num_cells_->toolTip());

  capacity_ = new qt::SpinBox();
  capacity_->setToolTip("The amount of electric charge that can be drawn from the battery.");
  capacity_->setMinimum(1);
  capacity_->setValue(5000);
  capacity_->setSuffix(" mAh");
  form->addRow(kCapacityKey, capacity_);
  form->labelForField(capacity_)->setToolTip(capacity_->toolTip());

  C_cont_ = new qt::SpinBox();
  C_cont_->setToolTip("The maximum continuous discharge current that the battery can provide.");
  C_cont_->setMinimum(1);
  C_cont_->setValue(50);
  C_cont_->setSuffix(" /h");
  form->addRow(kCContKey, C_cont_);
  form->labelForField(C_cont_)->setToolTip(C_cont_->toolTip());

  registance_ = new qt::SpinBox();
  registance_->setToolTip("Internal resistance value per cell.");
  registance_->setMinimum(0);
  registance_->setValue(3);
  registance_->setSuffix(" mΩ");
  form->addRow(kRegistanceKey, registance_);
  form->labelForField(registance_)->setToolTip(registance_->toolTip());
}

const char* BatteryWidget_LiPo::name() const
{
  return "Lithium Polymer Battery (LiPo)";
}

bool BatteryWidget_LiPo::isValid()
{
  return true;
}

YAML::Node BatteryWidget_LiPo::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNumCellsKey] = num_cells_->value();
  node[kCapacityKey] = capacity_->value();
  node[kCContKey] = C_cont_->value();
  node[kRegistanceKey] = registance_->value();

  return node;
}

void BatteryWidget_LiPo::load(const YAML::Node& node)
{
  num_cells_->setValue(node[kNumCellsKey].as<int>());
  capacity_->setValue(node[kCapacityKey].as<int>());
  C_cont_->setValue(node[kCContKey].as<int>());
  registance_->setValue(node[kRegistanceKey].as<int>());
}

double BatteryWidget_LiPo::nominalVoltage()
{
  return num_cells_->value() * kNominalVoltagePerCell;
}

double BatteryWidget_LiPo::maxVoltage()
{
  return num_cells_->value() * kMaxVoltagePerCell;
}

double BatteryWidget_LiPo::sagVoltage()
{
  return num_cells_->value() * kSagVoltagePerCell;
}

double BatteryWidget_LiPo::maxCurrent()
{
  return capacity_->value() * C_cont_->value() * 1e-3;
}

double BatteryWidget_LiPo::capacity()
{
  return capacity_->value() * 3600 * 1e-3;
}

double BatteryWidget_LiPo::internalRegistance()
{
  return num_cells_->value() * registance_->value() * 1e-3;
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
