#include "tobas_flight_log_gui/flight_log.hpp"

#include <QDebug>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace gui
{
namespace log
{
FlightLogWidget::FlightLogWidget(rclcpp::Node::SharedPtr node)
{
  recorder_ = new FlightLogRecorderWidget(node);
  logs_fc_ = new FlightLogsWidgetFC(node);
  logs_gcs_ = new FlightLogsWidgetGCS();
  log_viewer_ = new FlightLogViewerWidget();

  // Layout
  const auto log_cols = new QHBoxLayout();
  log_cols->addWidget(logs_fc_, 1);
  log_cols->addWidget(logs_gcs_, 1);

  const auto left_rows = new QVBoxLayout();
  left_rows->addWidget(recorder_, 1);
  left_rows->addLayout(log_cols, 1);

  const auto root_cols = new QHBoxLayout();
  root_cols->addLayout(left_rows, 1);
  root_cols->addWidget(log_viewer_, 1);

  setLayout(root_cols);

  // Connection
  connect(logs_fc_, &FlightLogsWidgetFC::logDownloaded, this, &self::onLogDownloaded);
  connect(logs_gcs_, &FlightLogsWidgetGCS::logSelected, this, &self::onLogSelected);
}

void FlightLogWidget::reset()
{
  recorder_->reset();
}

void FlightLogWidget::updateNamespace(const std::string& ns)
{
  recorder_->updateNamespace(ns);
}

void FlightLogWidget::onLogDownloaded(const QString& log_name)
{
  if (logs_gcs_->findLog(log_name)) {
    qInfo() << "\"" << log_name << "\" already exists in the GCS log list.";
    return;
  }

  logs_gcs_->addLog(log_name);
  logs_gcs_->sortLogs();
}

void FlightLogWidget::onLogSelected(const QString& log_name)
{
  log_viewer_->setLogName(log_name);
}
}  // namespace log
}  // namespace gui
