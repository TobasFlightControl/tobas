// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/fields/stop_at_waypoint.hpp"

#include <QHBoxLayout>

#define PASS_THROUGH_LABEL "Pass Through"
#define STOP_AT_WAYPOINT_LABEL "Stop at Waypoint"

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
StopAtWaypointWidget::StopAtWaypointWidget()
{
  combobox_ = new qt::ComboBox();
  combobox_->addItem(PASS_THROUGH_LABEL);
  combobox_->addItem(STOP_AT_WAYPOINT_LABEL);

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(combobox_);

  connect(combobox_, qOverload<int>(&QComboBox::currentIndexChanged), this, &BaseFieldWidget::updated);
}

const char* StopAtWaypointWidget::label() const
{
  return "Waypoint Behavior";
}

bool StopAtWaypointWidget::getValue() const
{
  return combobox_->currentText() == STOP_AT_WAYPOINT_LABEL;
}

void StopAtWaypointWidget::setValue(bool value)
{
  combobox_->setCurrentText(value ? STOP_AT_WAYPOINT_LABEL : PASS_THROUGH_LABEL);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
