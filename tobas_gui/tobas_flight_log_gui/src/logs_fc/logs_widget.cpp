// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/logs_fc/logs_widget.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/path.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/widgets/progress_dialog.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_std_tools/check.hpp>

#include "tobas_flight_log_gui/constants.hpp"
#include "tobas_flight_log_gui/logs_fc/log_item.hpp"

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace log
{
FlightLogsWidgetFC::FlightLogsWidgetFC(rclcpp::Node::SharedPtr node)
  : ssh_client_(node), spinner_(Qt::WindowModal, this)
{
  read_button_ = new QPushButton("Read");
  clean_button_ = new QPushButton("Clean");

  read_button_->setFixedSize(kButtonWidth, kButtonHeight);
  clean_button_->setFixedSize(kButtonWidth, kButtonHeight);

  read_button_->setEnabled(false);  // ホストが決まらないとSSH接続できないためTBSが読み込まれるまでは無効化
  clean_button_->setEnabled(false);

  log_list_ = new qt::ListWidget();
  log_list_->setSelectionMode(QListWidget::NoSelection);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(new qt::Label("Flight Controller", kPSize1, QFont::Bold));
  cols->addStretch();
  cols->addWidget(read_button_);
  cols->addWidget(clean_button_);

  const auto rows = new QVBoxLayout();
  rows->addLayout(cols);
  rows->addWidget(log_list_);

  setLayout(rows);

  // Connection
  connect(read_button_, &QPushButton::clicked, this, &self::onReadButtonClicked);
  connect(clean_button_, &QPushButton::clicked, this, &self::onCleanButtonClicked);
}

void FlightLogsWidgetFC::onProjectLoaded()
{
  read_button_->setEnabled(true);
}

void FlightLogsWidgetFC::addLog(const QString& log_name)
{
  const auto list_item = new qt::ListWidgetItem();
  list_item->setSizeHint(QSize(0, kListItemHeight));
  list_item->setData(Qt::UserRole, log_name);
  log_list_->addItem(list_item);

  const auto widget = new FlightLogItemWidgetFC(log_name);
  connect(widget, &FlightLogItemWidgetFC::downloadButtonClicked, this, &self::onDownloadButtonClicked);
  connect(widget, &FlightLogItemWidgetFC::deleteButtonClicked, this, &self::onDeleteButtonClicked);
  log_list_->setItemWidget(list_item, widget);

  sortLogs();
}

void FlightLogsWidgetFC::removeLog(const QString& log_name)
{
  const auto list_item = findLog(log_name);
  TOBAS_CHECK(list_item);
  log_list_->remove(list_item);
}

QListWidgetItem* FlightLogsWidgetFC::findLog(const QString& log_name)
{
  for (int row = 0; row < log_list_->count(); ++row) {
    const auto list_item = log_list_->item(row);
    const auto log_widget = qt::qConstPointerCast<FlightLogItemWidgetFC>(log_list_->itemWidget(list_item));

    if (log_widget->logName() == log_name) {
      return list_item;
    }
  }

  return nullptr;
}

void FlightLogsWidgetFC::clearLogs()
{
  log_list_->clear();
}

void FlightLogsWidgetFC::sortLogs()
{
  log_list_->sortItems();
}

void FlightLogsWidgetFC::onReadButtonClicked()
{
  std::vector<std::string> log_names;

  spinner_.start();
  const auto res = ssh_client_.list(kRosbagDirRoot, log_names);
  spinner_.stop();

  if (res != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, ssh_client_.errorMessage());
    return;
  }

  clearLogs();

  if (log_names.empty()) {
    qt::qWarnBox(this, "There are no flight logs saved on the flight controller.");
    return;
  }

  for (const auto& log_name : log_names) {
    addLog(QString::fromStdString(log_name));
  }

  sortLogs();

  clean_button_->setEnabled(true);
}

void FlightLogsWidgetFC::onCleanButtonClicked()
{
  if (!qt::yesOrNo(this, "Do you want to clean all the flight logs saved in the FC?", qt::WARN)) {
    return;
  }

  spinner_.start();
  const auto res = ssh_client_.execute("rm -rf " + std::string(kRosbagDirRoot) + "/*", true);
  spinner_.stop();

  if (res != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, ssh_client_.errorMessage());
    return;
  }

  clearLogs();
}

void FlightLogsWidgetFC::onDownloadButtonClicked(const QString& log_name)
{
  const auto rosbag_path = ros2::expandUser(kRosbagDirHome) / log_name.toStdString();

  if (fs::exists(rosbag_path)) {
    if (qt::yesOrNo(this, QString(rosbag_path.c_str()) + " already exists. Do you want to overwrite it?", qt::WARN)) {
      fs::remove_all(rosbag_path);
    }
    else {
      return;
    }
  }

  const auto remote_rosbag_path = fs::path(kRosbagDirRoot) / log_name.toStdString();
  const auto local_pardir = ros2::expandUser(kRosbagDirHome);

  if (!fs::is_directory(local_pardir)) {
    TOBAS_CHECK(fs::create_directories(local_pardir));
  }

  qt::ProgressDialog progress("Downloading Flight Log", 100, this);
  progress.setLabelText("Downloading flight log from the vehicle...");
  progress.setCancelButton(nullptr);

  const auto callback = [&progress, &log_name](uint64_t total_size, uint64_t transferred)
  {
    if (total_size <= 0) {
      qWarning() << "The total size of" << log_name << "is 0 bytes.";
      return;
    }
    const auto rate = 100 * transferred / total_size;
    progress.setStep(rate);
  };

  progress.show();
  const auto res = ssh_client_.scpGet(remote_rosbag_path, local_pardir, callback);
  progress.close();

  if (res != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, ssh_client_.errorMessage());
    return;
  }

  Q_EMIT logDownloaded(log_name);
}

void FlightLogsWidgetFC::onDeleteButtonClicked(const QString& log_name)
{
  const auto log_path = ros2::expandUser(kRosbagDirHome) / log_name.toStdString();

  if (!qt::yesOrNo(this, "Do you want to delete flight log \"" + log_name + "\"?", qt::WARN)) {
    return;
  }

  const auto rosbag_path = fs::path(kRosbagDirRoot) / log_name.toStdString();

  spinner_.start();
  const auto res = ssh_client_.execute("rm -rf " + rosbag_path.string(), true);
  spinner_.stop();

  if (res != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, ssh_client_.errorMessage());
    return;
  }

  removeLog(log_name);
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
