// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>
#include <QSettings>

#include <tobas_qt_tools/widgets/list_widget.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>

namespace tobas::gui::log
{
class FlightLogsWidgetGCS : public QWidget
{
  Q_OBJECT

  using self = FlightLogsWidgetGCS;

Q_SIGNALS:
  void logSelected(const QString& _log_name);
  void logDeselected();

public:
  explicit FlightLogsWidgetGCS();

  void addLog(const QString& _log_name);
  void removeLog(const QString& _log_name);
  QListWidgetItem* findLog(const QString& _log_name);

  void clearLogs();

private:
  QSettings settings_store_;

  QPushButton* read_button_;
  QPushButton* clean_button_;

  qt::ListWidget* log_list_;

  qt::WaitSpinnerWidget spinner_;

  QString currentLogName() const;
  void setCurrentLogName(const QString& _log_name);

  void sortLogs();

private Q_SLOTS:
  void onReadButtonClicked();
  void onCleanButtonClicked();
  void onExportButtonClicked(const QString& _log_name);
  void onDeleteButtonClicked(const QString& _log_name);
  void onListItemChanged(QListWidgetItem* _item);
};
}  // namespace tobas::gui::log
