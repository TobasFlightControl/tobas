// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/commands/base.hpp"

#include <cmath>

#include <QHBoxLayout>
#include <QLabel>
#include <QStackedWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
BaseCommandWidget::BaseCommandWidget()
{
  const auto root_rows = new QVBoxLayout();
  setLayout(root_rows);

  const auto header_cols = new QHBoxLayout();
  root_rows->addLayout(header_cols);

  label_ = new QLabel();
  label_->setFont(qt::DefaultFont(12, QFont::Bold));
  header_cols->addWidget(label_);

  header_cols->addStretch();

  delete_button_ = new QPushButton("Delete");
  header_cols->addWidget(delete_button_);
  connect(delete_button_, &QPushButton::clicked, this, &self::onDeleteButtonClicked);

  const auto scroll_area = new qt::ScrollArea();
  root_rows->addWidget(scroll_area);

  const auto field_rows = new QVBoxLayout();
  scroll_area->setLayout(field_rows);

  form_ = new qt::FormLayout();
  field_rows->addLayout(form_);

  field_rows->addStretch(1);  // Minimize the form widget.

  // Use a timer callback because pure virtual functions cannot be called from the base-class constructor.
  QTimer::singleShot(0, this, &self::initialize);
}

void BaseCommandWidget::addField(field::BaseFieldWidget* _widget, bool _overridable)
{
  const auto checkbox = new qt::CheckBox(_widget->label());
  checkbox->setDisabledTextNormal();
  checkboxes_[_widget] = checkbox;

  const auto stacked = new QStackedWidget();
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

  form_->addVAlignedRow(checkbox, stacked);

  connect(checkbox, &QCheckBox::toggled, [stacked](bool _checked) { stacked->setCurrentIndex((int)_checked); });
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

void BaseCommandWidget::initialize()
{
  label_->setText(name());
}

void BaseCommandWidget::onFieldUpdated()
{
  Q_EMIT updated();
}

void BaseCommandWidget::onDeleteButtonClicked()
{
  Q_EMIT deleteButtonClicked();
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
