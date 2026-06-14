// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

#include <tobas_property_client/property_client.hpp>
#include <tobas_qt_tools/widgets/list_widget.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
class FlightLogsWidgetGCS : public QWidget
{
  Q_OBJECT

  using self = FlightLogsWidgetGCS;
  using super = QWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;
  static constexpr int kListItemHeight = 40;

  static constexpr char kLastOpenedDirKey[] = "last_opened_dir";

Q_SIGNALS:
  void logSelected(const QString& log_name);
  void logDeselected();

public:
  explicit FlightLogsWidgetGCS(rclcpp::Node::SharedPtr node);

  void addLog(const QString& log_name);
  void removeLog(const QString& log_name);
  QListWidgetItem* findLog(const QString& log_name);

  void clearLogs();

private:
  ptree::PropertyClient property_client_;

  QPushButton* read_button_;
  QPushButton* clean_button_;

  qt::ListWidget* log_list_;

  qt::WaitSpinnerWidget spinner_;

  QString currentLogName() const;
  void setCurrentLogName(const QString& log_name);

  void sortLogs();

private Q_SLOTS:
  void onReadButtonClicked();
  void onCleanButtonClicked();
  void onExportButtonClicked(const QString& log_name);
  void onDeleteButtonClicked(const QString& log_name);
  void onListItemChanged(QListWidgetItem* item);
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
