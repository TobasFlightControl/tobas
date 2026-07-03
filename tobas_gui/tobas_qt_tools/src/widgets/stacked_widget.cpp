// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/stacked_widget.hpp"

#include <QApplication>

namespace tobas
{
namespace qt
{
void StackedWidget::clear()
{
  while (count() > 0) {
    const auto tar_widget = widget(0);
    removeWidget(tar_widget);
    tar_widget->deleteLater();
  }
}

void StackedWidget::setCurrentIndex(int index)
{
  // Update the index.
  super::setCurrentIndex(index);

  // Update the Qt event loop.
  QApplication::processEvents();

  // Update the screen.
  update();
}
}  // namespace qt
}  // namespace tobas
