// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>

namespace tobas
{
namespace qt
{
class MainWidget : public QWidget
{
  Q_OBJECT

public:
  explicit MainWidget(const QString& _title, const QString& _icon_path, QWidget* _widget);

protected:
  void closeEvent(QCloseEvent* _event) override;

private:
  QWidget* widget_;
  bool geometry_saved_ = false;

  void restoreWindowGeometry();
  void saveWindowGeometry();
};
}  // namespace qt
}  // namespace tobas
