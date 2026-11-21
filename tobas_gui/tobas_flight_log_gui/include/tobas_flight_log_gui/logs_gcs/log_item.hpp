// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QEvent>
#include <QLabel>
#include <QMenu>
#include <QPaintEvent>
#include <QWidget>

namespace tobas
{
namespace gui
{
namespace log
{
class FlightLogItemWidgetGCS : public QWidget
{
  Q_OBJECT

  using super = QWidget;

Q_SIGNALS:
  void showButtonClicked(const QString& log_name);
  void exportButtonClicked(const QString& log_name);
  void deleteButtonClicked(const QString& log_name);

public:
  explicit FlightLogItemWidgetGCS(const QString& log_name);

  QString logName() const;

private:
  QLabel* log_name_;
  QMenu* actions_menu_;

private Q_SLOTS:
  void onExportButtonClicked();
  void onDeleteButtonClicked();

private:
  bool eventFilter(QObject* watched, QEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

  bool isCursorOverActions() const;
  void showActionsMenu();
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
