// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/flight_log.hpp"

#include <QDebug>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace tobas::gui::log
{
FlightLogWidget::FlightLogWidget(const rqt::RosQtBridge& _bridge)
{
  recorder_ = new FlightLogRecorderWidget(_bridge);
  logs_fc_ = new FlightLogsWidgetFC();
  logs_gcs_ = new FlightLogsWidgetGCS();
  log_viewer_ = new FlightLogViewerWidget();

  // Layout
  const auto log_cols = new QHBoxLayout();
  log_cols->addWidget(logs_fc_, 1);
  log_cols->addWidget(logs_gcs_, 1);

  const auto ctrl_rows = new QVBoxLayout();
  ctrl_rows->addWidget(recorder_);
  ctrl_rows->addSpacing(30);
  ctrl_rows->addLayout(log_cols);

  const auto root_cols = new QHBoxLayout();
  root_cols->addLayout(ctrl_rows, 1);
  root_cols->addWidget(log_viewer_, 2);

  setLayout(root_cols);

  // Connection
  connect(recorder_, &FlightLogRecorderWidget::recordFinished, this, &self::onRecordFinished);
  connect(logs_fc_, &FlightLogsWidgetFC::logDownloaded, this, &self::onLogDownloaded);
  connect(logs_gcs_, &FlightLogsWidgetGCS::logSelected, this, &self::onLogSelected);
  connect(logs_gcs_, &FlightLogsWidgetGCS::logDeselected, this, &self::onLogDeselected);
}

void FlightLogWidget::reset()
{
  recorder_->reset();
  logs_fc_->reset();
}

void FlightLogWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns)
{
  recorder_->initializeRosInterfaces(_node, _ns);
  logs_fc_->initializeRosInterfaces(_node, _ns);
}

void FlightLogWidget::clearRosInterfaces()
{
  recorder_->clearRosInterfaces();
  logs_fc_->clearRosInterfaces();
}

void FlightLogWidget::onRecordFinished(const QString& _log_name, bool _is_real)
{
  qDebug().nospace() << "FlightLogWidget::onRecordFinished(" << _log_name << ", " << _is_real << ")";

  if (logs_fc_->findLog(_log_name)) {
    qWarning() << _log_name << "already exists in the FC log list.";
    return;
  }

  if (_is_real) {
    logs_fc_->addLog(_log_name);
  }
  else {
    logs_gcs_->addLog(_log_name);
  }
}

void FlightLogWidget::onLogDownloaded(const QString& _log_name)
{
  qDebug().nospace() << "FlightLogWidget::onLogDownloaded(" << _log_name << ")";

  if (logs_gcs_->findLog(_log_name)) {
    qInfo() << _log_name << "already exists in the GCS log list.";
    return;
  }

  logs_gcs_->addLog(_log_name);
}

void FlightLogWidget::onLogSelected(const QString& _log_name)
{
  qDebug().nospace() << "FlightLogWidget::onLogSelected(" << _log_name << ")";

  log_viewer_->setLogName(_log_name);
}

void FlightLogWidget::onLogDeselected()
{
  qDebug() << "FlightLogWidget::onLogDeselected";

  log_viewer_->reset();
}
}  // namespace tobas::gui::log
