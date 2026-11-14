// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QEvent>
#include <QLabel>
#include <QMenu>
#include <QPaintEvent>
#include <QTimer>
#include <QWidget>

namespace tobas::gui::log
{
class FlightLogItemWidgetFC : public QWidget
{
  Q_OBJECT

  using self = FlightLogItemWidgetFC;
  using super = QWidget;

Q_SIGNALS:
  void downloadButtonClicked(const QString& _log_name);
  void deleteButtonClicked(const QString& _log_name);

public:
  explicit FlightLogItemWidgetFC(const QString& _log_name);

  QString logName() const;

private:
  QLabel* log_name_;
  QMenu* actions_menu_;
  QTimer* close_menu_timer_;

  bool eventFilter(QObject* _watched, QEvent* _event) override;
  void paintEvent(QPaintEvent* _event) override;

  bool isCursorOverActions() const;
  void showActionsMenu();

private Q_SLOTS:
  void onCloseTimerTimeout();
};
}  // namespace tobas::gui::log
