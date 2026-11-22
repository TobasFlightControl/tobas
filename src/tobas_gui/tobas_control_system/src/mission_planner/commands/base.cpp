// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/commands/base.hpp"

#include <cmath>
#include <functional>

#include <QLabel>
#include <QLayout>
#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/scroll_area.hpp>

namespace tobas::gui::ctrl
{
BaseCommandWidget::BaseCommandWidget()
{
  const auto root_rows = new QVBoxLayout(this);
  root_rows->setContentsMargins(0, 0, 0, 0);

  const auto scroll_area = new qt::ScrollArea();
  root_rows->addWidget(scroll_area);

  const auto content = new QWidget();
  const auto field_rows = new QVBoxLayout(content);
  scroll_area->setWidget(content);

  form_ = new qt::FormLayout();
  form_->setVerticalSpacing(4);
  field_rows->addLayout(form_);

  field_rows->addStretch(1);  // Minimize the form widget.
}

void BaseCommandWidget::addField(field::BaseFieldWidget* _widget, bool _overridable)
{
  const auto checkbox = new qt::CheckBox(_widget->label());
  checkbox->setDisabledTextNormal();
  auto tooltip = _widget->toolTip();
  if (_overridable) {
    tooltip += "\nSelect this option to override the project default for this command. "
               "Clear it to use the default configured in Setup Assistant.";
  }
  checkbox->setToolTip(tooltip);
  checkboxes_[_widget] = checkbox;

  const auto stacked = new QStackedWidget();
  stacked->setToolTip(tooltip);
  stacked->addWidget(new QLabel("    Project Default"));
  stacked->addWidget(_widget);

  if (_overridable) {
    checkbox->setChecked(false);
    stacked->setCurrentIndex(0);
  }
  else {
    checkbox->setChecked(true);
    checkbox->setEnabled(false);
    stacked->setCurrentIndex(1);
  }

  _widget->layout()->setContentsMargins(-1, 0, -1, 0);
  form_->addVAlignedRow(checkbox, stacked);

  connect(checkbox, &QCheckBox::toggled, std::bind(&self::onCheckBoxToggled, this, std::placeholders::_1, stacked));
  connect(_widget, &field::BaseFieldWidget::updated, this, &self::onFieldUpdated);

  ++row_;
}

bool BaseCommandWidget::isChecked(field::BaseFieldWidget* _widget) const
{
  return checkboxes_[_widget]->isChecked();
}

void BaseCommandWidget::setChecked(field::BaseFieldWidget* _widget, bool _checked)
{
  checkboxes_[_widget]->setChecked(_checked);
}

double BaseCommandWidget::getValueOrDefault(field::FieldWidget<double>* _widget) const
{
  return isChecked(_widget) ? _widget->getValue() : NAN;
}

void BaseCommandWidget::setValueOrDefault(field::FieldWidget<double>* _widget, double _value)
{
  if (std::isfinite(_value)) {
    setChecked(_widget, true);
    _widget->setValue(_value);
  }
  else {
    setChecked(_widget, false);
  }
}

void BaseCommandWidget::onCheckBoxToggled(bool _checked, QStackedWidget* _stack)
{
  const auto index = static_cast<int>(_checked);
  _stack->setCurrentIndex(index);
}

void BaseCommandWidget::onFieldUpdated()
{
  Q_EMIT updated();
}
}  // namespace tobas::gui::ctrl
