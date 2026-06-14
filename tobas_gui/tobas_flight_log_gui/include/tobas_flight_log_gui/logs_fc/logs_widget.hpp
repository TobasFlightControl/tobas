// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

#include <tobas_gui_common/ssh_client.hpp>
#include <tobas_qt_tools/widgets/list_widget.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
class FlightLogsWidgetFC : public QWidget
{
  Q_OBJECT

  using self = FlightLogsWidgetFC;
  using super = QWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;
  static constexpr int kListItemHeight = 40;

Q_SIGNALS:
  void logDownloaded(const QString& log_name);

public:
  explicit FlightLogsWidgetFC(rclcpp::Node::SharedPtr node);

  void onProjectLoaded();

  void addLog(const QString& log_name);
  void removeLog(const QString& log_name);
  QListWidgetItem* findLog(const QString& log_name);

  void clearLogs();

private:
  cmn::SshClientWrapper ssh_client_;

  QPushButton* read_button_;
  QPushButton* clean_button_;

  qt::WaitSpinnerWidget spinner_;

  qt::ListWidget* log_list_;

  void sortLogs();

private Q_SLOTS:
  void onReadButtonClicked();
  void onCleanButtonClicked();
  void onDownloadButtonClicked(const QString& log_name);
  void onDeleteButtonClicked(const QString& log_name);
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
