// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/layouts/form_layout.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QWidget>

namespace tobas
{
namespace qt
{
void FormLayout::addVAlignedRow(QWidget* _label, QWidget* _field)
{
  const auto label_layout = new QHBoxLayout();
  label_layout->addWidget(_label, 0, Qt::AlignLeft | Qt::AlignVCenter);

  const auto label_widget = new QWidget();
  label_widget->setLayout(label_layout);

  addRow(label_widget, _field);
}

void FormLayout::addVAlignedRow(QWidget* _label, QLayout* _field)
{
  const auto field_widget = new QWidget();
  field_widget->setLayout(_field);

  addVAlignedRow(_label, field_widget);
}

void FormLayout::addVAlignedRow(const QString& _label_text, QWidget* _field)
{
  addVAlignedRow(new QLabel(_label_text), _field);
}

void FormLayout::addVAlignedRow(const QString& _label_text, QLayout* _field)
{
  addVAlignedRow(new QLabel(_label_text), _field);
}

void FormLayout::addStretch()
{
  addItem(new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding));
}

void FormLayout::clear()
{
  while (rowCount() > 0) {
    removeRow(0);
  }
}

QWidget* FormLayout::getLabel(int _row)
{
  const auto item = itemAt(_row, QFormLayout::LabelRole);
  return item->widget();
}

QWidget* FormLayout::getWidget(int _row)
{
  const auto item = itemAt(_row, QFormLayout::FieldRole);
  return item->widget();
}

void FormLayout::setEnabled(int _row, bool _enabled)
{
  getLabel(_row)->setEnabled(_enabled);
  getWidget(_row)->setEnabled(_enabled);
}

void FormLayout::setVisible(int _row, bool _visible)
{
  getLabel(_row)->setVisible(_visible);
  getWidget(_row)->setVisible(_visible);
}
}  // namespace qt
}  // namespace tobas
