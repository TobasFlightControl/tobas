// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_system.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

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
PropulsionSystemWidget::PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf)
{
  battery = new BatteryWidget();
  units = new PropulsionUnitsWidget(node, uadf);

  // Layout
  const auto battery_rows = new QVBoxLayout();
  qt::addWidgetCenter(new qt::Label("Battery", cmn::kTitlePSize, QFont::Bold), battery_rows);
  battery_rows->addWidget(battery);

  const auto units_rows = new QVBoxLayout();
  qt::addWidgetCenter(new qt::Label("Propulsion Units", cmn::kTitlePSize, QFont::Bold), units_rows);
  units_rows->addWidget(units);

  const auto cols = new QHBoxLayout();
  cols->addLayout(battery_rows, 1);
  cols->addLayout(units_rows, 1);

  setLayout(cols);
}

const char* PropulsionSystemWidget::name() const
{
  return "Electric Propulsion System";
}

void PropulsionSystemWidget::updateInternalDataStructures()
{
  units->updateInternalDataStructures();
}

bool PropulsionSystemWidget::isValid()
{
  if (!battery->isValid()) {
    return false;
  }

  if (!units->isValid()) {
    return false;
  }

  return true;
}

YAML::Node PropulsionSystemWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kBatteryTitle] = battery->dump();
  node[kPropulsionUnitTitle] = units->dump();

  return node;
}

void PropulsionSystemWidget::load(const YAML::Node& node)
{
  battery->load(node[kBatteryTitle]);
  units->load(node[kPropulsionUnitTitle]);
}

PropulsionSystem PropulsionSystemWidget::type() const
{
  return PropulsionSystem::kElectric;
}

int PropulsionSystemWidget::numUnits() const
{
  return units->count();
}

QString PropulsionSystemWidget::linkName(int index) const
{
  return units->linkName(index);
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
