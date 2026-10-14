// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_simulation_gui/simulation_settings/world/world.hpp"

#include <QButtonGroup>
#include <QDebug>
#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

#include "tobas_simulation_gui/simulation_settings/world/custom_world.hpp"
#include "tobas_simulation_gui/simulation_settings/world/standard_world.hpp"

namespace fs = std::filesystem;

namespace tobas::gui::sim
{
WorldWidget::WorldWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto label = new qt::Label("World", cmn::kLabelPSize, QFont::Bold);
  rows->addWidget(label);

  const auto form = new qt::FormLayout();
  rows->addLayout(form);

  const auto btn_group = new QButtonGroup();
  btn_group->setExclusive(true);

  addRow(form, btn_group, new WorldWidget_Standard(), "Standard World");
  addRow(form, btn_group, new CustomWorldWidget(), "Custom World");

  // Default
  buttons_.at(0)->setChecked(true);
  updateEnabled();

  connect(btn_group, &QButtonGroup::idClicked, this, &self::onButtonGroupIdClicked);
}

fs::path WorldWidget::worldPath() const
{
  return widgets_.at(findCurrentRow())->worldPath();
}

void WorldWidget::addRow(qt::FormLayout* _form, QButtonGroup* _btn_group, BaseWorldWidget* _widget, const QString& _label)
{
  const auto button = new QRadioButton(_label);
  _btn_group->addButton(button, rowCount());
  _form->addVAlignedRow(button, _widget);

  buttons_.append(button);
  widgets_.append(_widget);
}

void WorldWidget::updateEnabled()
{
  for (int i = 0; i < rowCount(); ++i) {
    widgets_.at(i)->setEnabled(buttons_.at(i)->isChecked());
  }
}

int WorldWidget::rowCount() const
{
  assert(buttons_.size() == widgets_.size());
  return buttons_.size();
}

int WorldWidget::findCurrentRow() const
{
  for (int i = 0; i < rowCount(); ++i) {
    if (buttons_.at(i)->isChecked()) {
      return i;
    }
  }

  qWarning() << "No button is checked.";
  return -1;
}

void WorldWidget::onButtonGroupIdClicked()
{
  updateEnabled();
}
}  // namespace tobas::gui::sim
