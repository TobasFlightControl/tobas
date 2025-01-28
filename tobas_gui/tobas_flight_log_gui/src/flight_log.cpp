#include <QHBoxLayout>

#include "tobas_flight_log_gui/flight_log.hpp"

namespace gui
{
namespace log
{
FlightLogWidget::FlightLogWidget(rclcpp::Node::SharedPtr node)
{
  recorder_ = new FlightLogRecorderWidget(node);
  logs_fc_ = new FlightLogsWidgetFC(node);
  logs_gcs_ = new FlightLogsWidgetGCS();

  const auto log_cols = new QHBoxLayout();
  log_cols->addWidget(logs_fc_, 3);
  log_cols->addStretch(1);  // TODO: Download Button
  log_cols->addWidget(logs_gcs_, 3);
  log_cols->addStretch();

  const auto rows = new QVBoxLayout();
  rows->addWidget(recorder_, 1);
  rows->addLayout(log_cols, 1);

  const auto cols = new QHBoxLayout();
  cols->addLayout(rows, 1);
  cols->addStretch(1);  // TODO: Viewer

  setLayout(cols);
}

void FlightLogWidget::updateNamespace(const std::string& ns)
{
  recorder_->updateNamespace(ns);
}
}  // namespace log
}  // namespace gui
