// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

#include <tobas_gui_common/ssh_client.hpp>
#include <tobas_qt_tools/widgets/list_widget.hpp>

namespace tobas::gui::log
{
class FlightLogsWidgetFC : public QWidget
{
  Q_OBJECT

  using self = FlightLogsWidgetFC;

Q_SIGNALS:
  void logDownloaded(const QString& _log_name);

public:
  explicit FlightLogsWidgetFC(rclcpp::Node::SharedPtr _node);

  void reset();

  void addLog(const QString& _log_name);
  void removeLog(const QString& _log_name);
  QListWidgetItem* findLog(const QString& _log_name);

  void clearLogs();

private:
  cmn::SshClientWrapper ssh_client_;

  QPushButton* read_button_;
  QPushButton* clean_button_;

  qt::ListWidget* log_list_;

  void sortLogs();

private Q_SLOTS:
  void onReadButtonClicked();
  void onCleanButtonClicked();
  void onDownloadButtonClicked(const QString& _log_name);
  void onDeleteButtonClicked(const QString& _log_name);
};
}  // namespace tobas::gui::log
