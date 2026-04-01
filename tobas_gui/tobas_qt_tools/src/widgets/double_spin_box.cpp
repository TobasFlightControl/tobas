// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/double_spin_box.hpp"

#include <QTimer>
#include <QWheelEvent>

namespace tobas
{
namespace qt
{
DoubleSpinBox::DoubleSpinBox(QWidget* parent) : super(parent)
{
  setMinimum(std::numeric_limits<double>::lowest());
  setMaximum(std::numeric_limits<double>::max());
}

void DoubleSpinBox::wheelEvent(QWheelEvent* event)
{
  event->ignore();
}

void DoubleSpinBox::focusInEvent(QFocusEvent* event)
{
  super::focusInEvent(event);
  QTimer::singleShot(0, this, &DoubleSpinBox::selectAll);
}
}  // namespace qt
}  // namespace tobas
