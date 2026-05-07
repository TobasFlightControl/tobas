// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/spin_box.hpp"

#include <QTimer>
#include <QWheelEvent>

namespace tobas
{
namespace qt
{
SpinBox::SpinBox(QWidget* parent) : super(parent)
{
  setMinimum(std::numeric_limits<int>::lowest());
  setMaximum(std::numeric_limits<int>::max());
}

void SpinBox::wheelEvent(QWheelEvent* event)
{
  event->ignore();
}

void SpinBox::focusInEvent(QFocusEvent* event)
{
  super::focusInEvent(event);
  QTimer::singleShot(0, this, &SpinBox::selectAll);
}
}  // namespace qt
}  // namespace tobas
