// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>
#include <QPushButton>

namespace tobas::gui::log
{
class FlightLogItemWidgetFC : public QWidget
{
  Q_OBJECT

  using self = FlightLogItemWidgetFC;

Q_SIGNALS:
  void downloadButtonClicked(const QString& _log_name);
  void deleteButtonClicked(const QString& _log_name);

public:
  explicit FlightLogItemWidgetFC(const QString& _log_name);

  QString logName() const;

private:
  QLabel* log_name_;
  QPushButton* download_button_;
  QPushButton* delete_button_;

private Q_SLOTS:
  void onDownloadButtonClicked();
  void onDeleteButtonClicked();
};
}  // namespace tobas::gui::log
