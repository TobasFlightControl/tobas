// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/flight_log.hpp"

#include <QDebug>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace tobas
{
namespace gui
{
namespace log
{
FlightLogWidget::FlightLogWidget(const rqt::RosQtBridge& bridge)
{
  recorder_ = new FlightLogRecorderWidget(bridge);
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
}

void FlightLogWidget::onProjectLoaded()
{
  logs_fc_->onProjectLoaded();
}

void FlightLogWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns)
{
  recorder_->initializeRosInterfaces(node, ns);
  logs_fc_->initializeRosInterfaces(node, ns);
}

void FlightLogWidget::clearRosInterfaces()
{
  recorder_->clearRosInterfaces();
  logs_fc_->clearRosInterfaces();
}

void FlightLogWidget::onRecordFinished(const QString& log_name, bool is_real)
{
  qDebug().nospace() << "FlightLogWidget::onRecordFinished(" << log_name << ", " << is_real << ")";

  if (logs_fc_->findLog(log_name)) {
    qWarning() << log_name << "already exists in the FC log list.";
    return;
  }

  if (is_real) {
    logs_fc_->addLog(log_name);
  }
  else {
    logs_gcs_->addLog(log_name);
  }
}

void FlightLogWidget::onLogDownloaded(const QString& log_name)
{
  qDebug().nospace() << "FlightLogWidget::onLogDownloaded(" << log_name << ")";

  if (logs_gcs_->findLog(log_name)) {
    qInfo() << log_name << "already exists in the GCS log list.";
    return;
  }

  logs_gcs_->addLog(log_name);
}

void FlightLogWidget::onLogSelected(const QString& log_name)
{
  qDebug().nospace() << "FlightLogWidget::onLogSelected(" << log_name << ")";

  log_viewer_->setLogName(log_name);
}

void FlightLogWidget::onLogDeselected()
{
  qDebug() << "FlightLogWidget::onLogDeselected";

  log_viewer_->reset();
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
