// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/main_widget.hpp"

#include <QCloseEvent>
#include <QCoreApplication>
#include <QIcon>
#include <QSettings>
#include <QVBoxLayout>

namespace
{
constexpr char kWindowGroup[] = "window";
constexpr char kGeometryKey[] = "geometry";

bool hasSettingsIdentity()
{
  return !QCoreApplication::organizationName().isEmpty() && !QCoreApplication::applicationName().isEmpty();
}
}  // namespace

namespace tobas::qt
{
MainWidget::MainWidget(const QString& _title, const QString& _icon_path, QWidget* _widget) : widget_(_widget)
{
  setWindowTitle(_title);

  if (!_icon_path.isEmpty()) {
    setWindowIcon(QIcon(_icon_path));
  }

  const auto rows = new QVBoxLayout(this);
  rows->setContentsMargins(0, 0, 0, 0);
  rows->addWidget(_widget);

  restoreWindowGeometry();
  connect(QCoreApplication::instance(), &QCoreApplication::aboutToQuit, this, &MainWidget::saveWindowGeometry);
}

void MainWidget::closeEvent(QCloseEvent* _event)
{
  if (widget_->close()) {
    saveWindowGeometry();
    _event->accept();
  }
  else {
    _event->ignore();
  }
}

void MainWidget::restoreWindowGeometry()
{
  if (!hasSettingsIdentity()) {
    return;
  }

  QSettings settings;
  settings.beginGroup(kWindowGroup);

  // Qt restores normal geometry and maximized state, adjusting windows to the available screens.
  restoreGeometry(settings.value(kGeometryKey).toByteArray());
}

void MainWidget::saveWindowGeometry()
{
  if (!hasSettingsIdentity()) {
    return;
  }

  QSettings settings;
  settings.beginGroup(kWindowGroup);

  settings.setValue(kGeometryKey, saveGeometry());

  settings.endGroup();
  settings.sync();
}
}  // namespace tobas::qt
