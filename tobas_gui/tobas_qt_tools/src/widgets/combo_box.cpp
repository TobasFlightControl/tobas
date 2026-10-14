// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/combo_box.hpp"

#include <QStandardItemModel>
#include <QWheelEvent>

#include "tobas_qt_tools/cast.hpp"

namespace tobas::qt
{
void ComboBox::wheelEvent(QWheelEvent* _event)
{
  _event->ignore();
}

bool ComboBox::contains(const QString& _text) const
{
  return findText(_text) >= 0;
}

void ComboBox::removeText(const QString& _text)
{
  const auto index = findText(_text);
  if (index < 0) {
    throw std::runtime_error("'" + _text.toStdString() + "' does not exist in the combo box choices.");
  }
  removeItem(index);
}

void ComboBox::setCurrentIndex(int _index)
{
  if (_index < 0 || count() <= _index) {
    throw std::runtime_error("Index " + std::to_string(_index) + " is out of range.");
  }
  super::setCurrentIndex(_index);
}

void ComboBox::setCurrentText(const QString& _text)
{
  const auto index = findText(_text);
  if (index < 0) {
    throw std::runtime_error("'" + _text.toStdString() + "' does not exist in the combo box choices.");
  }
  super::setCurrentIndex(index);
}

void ComboBox::sort()
{
  model()->sort(0, Qt::AscendingOrder);
}

void ComboBox::setItemEnabled(int _row, bool _enabled)
{
  const auto model = qt::qConstPointerCast<QStandardItemModel>(this->model());
  const auto item = model->item(_row);
  const auto cur_flags = item->flags();
  const auto new_flags = _enabled ? cur_flags | Qt::ItemIsEnabled : cur_flags & ~Qt::ItemIsEnabled;
  item->setFlags(new_flags);
}

void ComboBox::setItemEnabled(const QString& _text, bool _enabled)
{
  setItemEnabled(findText(_text), _enabled);
}
}  // namespace tobas::qt
