// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/spin_box.hpp"

#include <QTimer>
#include <QWheelEvent>

namespace tobas
{
namespace qt
{
SpinBox::SpinBox(QWidget* _parent) : super(_parent)
{
  setMinimum(std::numeric_limits<int>::lowest());
  setMaximum(std::numeric_limits<int>::max());
}

void SpinBox::wheelEvent(QWheelEvent* _event)
{
  _event->ignore();
}

void SpinBox::focusInEvent(QFocusEvent* _event)
{
  super::focusInEvent(_event);
  QTimer::singleShot(0, this, &SpinBox::selectAll);
}
}  // namespace qt
}  // namespace tobas
