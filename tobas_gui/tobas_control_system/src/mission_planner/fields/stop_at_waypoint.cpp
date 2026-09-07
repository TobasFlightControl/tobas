// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/fields/stop_at_waypoint.hpp"

#include <QHBoxLayout>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
namespace
{
constexpr char kPassThroughLabel[] = "Pass Through";
constexpr char kStopAtWaypointLabel[] = "Stop at Waypoint";
}  // namespace

StopAtWaypointWidget::StopAtWaypointWidget()
{
  combobox_ = new qt::ComboBox();
  combobox_->addItem(kPassThroughLabel);
  combobox_->addItem(kStopAtWaypointLabel);

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
  return combobox_->currentText() == kStopAtWaypointLabel;
}

void StopAtWaypointWidget::setValue(bool value)
{
  combobox_->setCurrentText(value ? kStopAtWaypointLabel : kPassThroughLabel);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
