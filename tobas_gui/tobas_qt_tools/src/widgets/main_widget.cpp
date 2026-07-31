// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/main_widget.hpp"

#include <QCloseEvent>
#include <QCoreApplication>
#include <QGuiApplication>
#include <QIcon>
#include <QSettings>
#include <QVBoxLayout>

namespace
{
constexpr char kWindowGroup[] = "window";
constexpr char kPositionKey[] = "position";
constexpr char kSizeKey[] = "size";

bool hasSettingsIdentity()
{
  return !QCoreApplication::organizationName().isEmpty() && !QCoreApplication::applicationName().isEmpty();
}
}  // namespace

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

  restoreWindowGeometry();
  connect(QCoreApplication::instance(), &QCoreApplication::aboutToQuit, this, &MainWidget::saveWindowGeometry);
}

void MainWidget::closeEvent(QCloseEvent* event)
{
  if (widget_->close()) {
    saveWindowGeometry();
    event->accept();
  }
  else {
    event->ignore();
  }
}

void MainWidget::restoreWindowGeometry()
{
  if (!hasSettingsIdentity()) {
    return;
  }

  QSettings settings;
  settings.beginGroup(kWindowGroup);

  const auto size = settings.value(kSizeKey).toSize();
  if (size.isValid()) {
    resize(size);
  }

  if (settings.contains(kPositionKey)) {
    const auto position = settings.value(kPositionKey).toPoint();
    if (QGuiApplication::screenAt(position)) {
      move(position);
    }
  }

  settings.endGroup();
}

void MainWidget::saveWindowGeometry()
{
  if (geometry_saved_ || !hasSettingsIdentity()) {
    return;
  }

  QSettings settings;
  settings.beginGroup(kWindowGroup);
  settings.setValue(kPositionKey, pos());
  settings.setValue(kSizeKey, size());
  settings.endGroup();
  settings.sync();
  geometry_saved_ = true;
}
}  // namespace qt
}  // namespace tobas
