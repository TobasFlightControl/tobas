// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>

namespace tobas::qt
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

  void restoreWindowGeometry();
  void saveWindowGeometry();
};
}  // namespace tobas::qt
