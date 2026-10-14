// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/double_spin_box.hpp"

#include <QTimer>
#include <QWheelEvent>

namespace tobas
{
namespace qt
{
DoubleSpinBox::DoubleSpinBox(QWidget* _parent) : super(_parent)
{
  setMinimum(std::numeric_limits<double>::lowest());
  setMaximum(std::numeric_limits<double>::max());
}

void DoubleSpinBox::wheelEvent(QWheelEvent* _event)
{
  _event->ignore();
}

void DoubleSpinBox::focusInEvent(QFocusEvent* _event)
{
  super::focusInEvent(_event);
  QTimer::singleShot(0, this, &DoubleSpinBox::selectAll);
}
}  // namespace qt
}  // namespace tobas
