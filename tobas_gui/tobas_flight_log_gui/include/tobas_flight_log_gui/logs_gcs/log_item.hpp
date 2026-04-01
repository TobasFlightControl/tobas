// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>
#include <QPushButton>

namespace tobas
{
namespace gui
{
namespace log
{
class FlightLogItemWidgetGCS : public QWidget
{
  Q_OBJECT

  using self = FlightLogItemWidgetGCS;
  using super = QWidget;

  static constexpr int kButtonWidth = 80;

Q_SIGNALS:
  void exportButtonClicked(const QString& log_name);
  void deleteButtonClicked(const QString& log_name);

public:
  explicit FlightLogItemWidgetGCS(const QString& log_name);

  QString logName() const;

private:
  QLabel* log_name_;
  QPushButton* export_button_;
  QPushButton* delete_button_;

private Q_SLOTS:
  void onExportButtonClicked();
  void onDeleteButtonClicked();
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
