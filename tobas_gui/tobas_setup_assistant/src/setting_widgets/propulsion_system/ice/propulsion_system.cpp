// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_system.hpp"

#include <QGridLayout>
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
namespace ice
{
PropulsionSystemWidget::PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf)
{
  engine = new EngineWidget(node);
  units = new PropulsionUnitsWidget(node, uadf);

  // Layout
  const auto grid = new QGridLayout();
  grid->setRowStretch(1, 1);
  grid->setColumnStretch(0, 1);
  grid->setColumnStretch(1, 1);
  grid->addWidget(new qt::Label(kEngineTitle, cmn::kLabelPSize, QFont::Bold), 0, 0);
  grid->addWidget(new qt::Label(kPropulsionUnitTitle, cmn::kLabelPSize, QFont::Bold), 0, 1);
  grid->addWidget(engine, 1, 0);
  grid->addWidget(units, 1, 1);
  setLayout(grid);
}

const char* PropulsionSystemWidget::name() const
{
  return "ICE Propulsion System";
}

void PropulsionSystemWidget::updateInternalDataStructures()
{
  units->updateInternalDataStructures();
}

bool PropulsionSystemWidget::isValid()
{
  if (!engine->isValid()) {
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

  node[kEngineTitle] = engine->dump();
  node[kPropulsionUnitTitle] = units->dump();

  return node;
}

void PropulsionSystemWidget::load(const YAML::Node& node)
{
  engine->load(node[kEngineTitle]);
  units->load(node[kPropulsionUnitTitle]);
}

PropulsionSystem PropulsionSystemWidget::type() const
{
  return PropulsionSystem::kIce;
}

int PropulsionSystemWidget::numUnits() const
{
  return units->count();
}

QString PropulsionSystemWidget::linkName(int index) const
{
  return units->linkName(index);
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
