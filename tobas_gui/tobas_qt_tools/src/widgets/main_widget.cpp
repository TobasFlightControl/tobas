// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/main_widget.hpp"

#include <csignal>

#include <QCloseEvent>
#include <QIcon>
#include <QVBoxLayout>

namespace tobas
{
namespace qt
{
MainWidget::MainWidget(const QString& title, const QString& icon_path, QWidget* widget) : widget_(widget)
{
  setWindowTitle(title);

  if (!icon_path.isEmpty()) {
    setWindowIcon(QIcon(icon_path));
  }

  const auto rows = new QVBoxLayout();
  setLayout(rows);

  rows->addWidget(widget);
}

void MainWidget::closeEvent(QCloseEvent* event)
{
  if (widget_->close()) {
    event->accept();
  }
  else {
    event->ignore();
  }
}
}  // namespace qt
}  // namespace tobas
