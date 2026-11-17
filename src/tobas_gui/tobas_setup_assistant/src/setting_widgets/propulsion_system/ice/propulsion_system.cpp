// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_system.hpp"

#include <QGridLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas::gui::sa::propulsion::ice
{
namespace
{
constexpr char kEngineTitle[] = "Engine";
constexpr char kPropulsionUnitTitle[] = "Propulsion Units";
}  // namespace

PropulsionSystemWidget::PropulsionSystemWidget(const uadf::Model& _uadf)
{
  engine = new EngineWidget();
  units = new PropulsionUnitsWidget(_uadf);

  // Layout
  const auto grid = new QGridLayout(this);
  grid->setRowStretch(1, 1);
  grid->setColumnStretch(0, 1);
  grid->setColumnStretch(1, 1);
  grid->addWidget(new qt::Label(kEngineTitle, cmn::kSubTitlePointSize, QFont::Bold), 0, 0);
  grid->addWidget(new qt::Label(kPropulsionUnitTitle, cmn::kSubTitlePointSize, QFont::Bold), 0, 1);
  grid->addWidget(engine, 1, 0);
  grid->addWidget(units, 1, 1);
}

const char* PropulsionSystemWidget::name() const
{
  return "ICE Propulsion System";
}

void PropulsionSystemWidget::updateInternalDataStructures()
{
  units->updateInternalDataStructures();
}

void PropulsionSystemWidget::setToDefaults()
{
  engine->setToDefaults();
  units->setToDefaults();
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

void PropulsionSystemWidget::load(const YAML::Node& _node)
{
  engine->load(_node[kEngineTitle]);
  units->load(_node[kPropulsionUnitTitle]);
}

PropulsionSystem PropulsionSystemWidget::type() const
{
  return PropulsionSystem::kIce;
}

int PropulsionSystemWidget::numUnits() const
{
  return units->count();
}

QString PropulsionSystemWidget::linkName(int _index) const
{
  return units->linkName(_index);
}
}  // namespace tobas::gui::sa::propulsion::ice
