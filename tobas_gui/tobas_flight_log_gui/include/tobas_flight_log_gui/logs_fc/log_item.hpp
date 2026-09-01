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
class FlightLogItemWidgetFC : public QWidget
{
  Q_OBJECT

  using self = FlightLogItemWidgetFC;
  using super = QWidget;

Q_SIGNALS:
  void downloadButtonClicked(const QString& log_name);
  void deleteButtonClicked(const QString& log_name);

public:
  explicit FlightLogItemWidgetFC(const QString& log_name);

  QString logName() const;

private:
  QLabel* log_name_;
  QPushButton* download_button_;
  QPushButton* delete_button_;

private Q_SLOTS:
  void onDownloadButtonClicked();
  void onDeleteButtonClicked();
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
