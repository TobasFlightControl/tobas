// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/recorder/recorder.hpp"

#include <filesystem>

#include <QDebug>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/path.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/thread.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_string_tools/core.hpp>

#include "tobas_flight_log_gui/constants.hpp"

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace log
{
FlightLogRecorderWidget::FlightLogRecorderWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge)
  : start_thread_(node), stop_thread_(node), spinner_(Qt::WindowModal, this)
{
  log_name_ = new qt::HistoryLineEdit();

  start_stop_button_ = new qt::ToggleButton("▶ Start Recording", "■ Stop Recording");
  start_stop_button_->setFixedSize(kButtonWidth, kButtonHeight);

  duration_ = new QLCDNumber(8);
  duration_->setSegmentStyle(QLCDNumber::Flat);

  file_size_ = new qt::HPositionBarWidget();
  file_size_->setLower(0);
  file_size_->setMinimum(0);

  message_count_ = new qt::FramedLabel();

  reset();

  // Layout
  const auto name_cols = new QHBoxLayout();
  name_cols->addWidget(new qt::Label("Log Name", kPSize2));
  name_cols->addWidget(log_name_);

  const auto ctrl_cols = new QHBoxLayout();
  ctrl_cols->addWidget(start_stop_button_);
  ctrl_cols->addStretch();
  ctrl_cols->addWidget(duration_);

  const auto state_form = new qt::FormLayout();
  state_form->addVAlignedRow(new qt::Label("File Size", kPSize2), file_size_);
  state_form->addVAlignedRow(new qt::Label("Message Count", kPSize2), message_count_);

  const auto root_rows = new QVBoxLayout();
  root_rows->addLayout(name_cols);
  root_rows->addLayout(ctrl_cols);
  root_rows->addLayout(state_form);

  setLayout(root_rows);

  // Connection
  connect(start_stop_button_, &qt::ToggleButton::checked, this, &self::onStartRequested);
  connect(start_stop_button_, &qt::ToggleButton::unchecked, this, &self::onStopRequested);
  connect(&bridge, &RosQtBridge::rosbagStateReceived, this, &self::rosbagStateCb, Qt::QueuedConnection);
}

void FlightLogRecorderWidget::reset()
{
  start_stop_button_->setChecked(false);
  start_stop_button_->setEnabled(false);

  clearRosbagStateViewerWidgets();

  rosbag_state_.reset();
}

void FlightLogRecorderWidget::updateNamespace(const std::string& ns)
{
  reset();

  start_thread_.setNamespace(ns);
  stop_thread_.setNamespace(ns);
}

void FlightLogRecorderWidget::clearRosbagStateViewerWidgets()
{
  duration_->display("00:00:00");

  file_size_->setUpper(0);
  file_size_->setCenterText("0 MB");

  message_count_->setText("0");
}

void FlightLogRecorderWidget::onStartRequested()
{
  // ファイル名をチェック
  const auto log_name = log_name_->text().toStdString();
  if (log_name.empty()) {
    qt::qWarnBox(this, "Please specify the name of log file.");
    start_stop_button_->setChecked(false);
    return;
  }
  if (!str::isValidFileName(log_name)) {
    qt::qWarnBox(this, "The name of the log file is invalid.");
    start_stop_button_->setChecked(false);
    return;
  }

  // ロガーの状態が取得できているかをチェック
  if (!rosbag_state_) {
    qt::qWarnBox(this, "Unable to start recording because the logger state has not been received yet.");
    start_stop_button_->setChecked(false);
    return;
  }

  start_thread_.setLogName(log_name);

  spinner_.start();
  const auto [success, message] = qt::startThreadAndWait(start_thread_, &RecordStartThread::finished);
  spinner_.stop();

  if (!success) {
    qt::qErrorBox(this, message);
    start_stop_button_->setChecked(false);
    return;
  }

  log_name_->setEnabled(false);
  clearRosbagStateViewerWidgets();

  qt::qInfoBox(this, "Flight log recording has started.");
}

void FlightLogRecorderWidget::onStopRequested()
{
  spinner_.start();
  const auto [success, message, log_path] = qt::startThreadAndWait(stop_thread_, &RecordStopThread::finished);
  spinner_.stop();

  if (!success) {
    qt::qErrorBox(this, message);
    start_stop_button_->setChecked(true);
    return;
  }

  const auto log_name = QFileInfo(log_path).fileName();
  const auto is_real = log_path.startsWith(kRosbagDirRoot);
  Q_EMIT recordFinished(log_name, is_real);

  // ログ名を履歴に追加してクリア
  log_name_->addHistory(log_name);
  log_name_->clear();

  clearRosbagStateViewerWidgets();

  qt::qInfoBox(this, "Flight log recording has stopped.");
}

void FlightLogRecorderWidget::rosbagStateCb(const tobas_msgs::msg::RosbagState::ConstSharedPtr& rosbag_state)
{
  // 現在のレコーダの状態によってウィジェットの状態を切り替えて有効化
  start_stop_button_->setChecked(rosbag_state->recording);
  start_stop_button_->setEnabled(true);

  if (rosbag_state->recording) {
    log_name_->setText(fs::path(rosbag_state->file_path).lexically_normal().filename().c_str());
    log_name_->setEnabled(false);

    const auto& total_secs = rosbag_state->duration.sec;
    const auto hours = total_secs / 3600;
    const auto minutes = (total_secs % 3600) / 60;
    const auto seconds = total_secs % 60;
    const auto hhmmss = QString("%1:%2:%3")
                          .arg(hours, 2, 10, QLatin1Char('0'))
                          .arg(minutes, 2, 10, QLatin1Char('0'))
                          .arg(seconds, 2, 10, QLatin1Char('0'));
    duration_->display(hhmmss);

    file_size_->setUpper(rosbag_state->file_size);
    file_size_->setMaximum(rosbag_state->file_size + rosbag_state->available_size);
    file_size_->setCenterText(QString::number(rosbag_state->file_size / 1'000'000) + " MB");

    message_count_->setText(QString::number(rosbag_state->message_count));
  }
  else {
    log_name_->setEnabled(true);
    clearRosbagStateViewerWidgets();
  }

  rosbag_state_ = rosbag_state;
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
