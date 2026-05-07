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

  using super = QWidget;

public:
  explicit MainWidget(const QString& title, const QString& icon_path, QWidget* widget);

protected:
  void closeEvent(QCloseEvent* event) override;

private:
  QWidget* widget_;
};
}  // namespace qt
}  // namespace tobas
