#include <QVBoxLayout>
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
  log_cols->addWidget(logs_fc_, 1);
  log_cols->addWidget(logs_gcs_, 1);

  const auto left_rows = new QVBoxLayout();
  left_rows->addWidget(recorder_, 1);
  left_rows->addLayout(log_cols, 1);

  const auto root_cols = new QHBoxLayout();
  root_cols->addLayout(left_rows, 1);
  root_cols->addStretch(1);  // TODO: Viewer

  setLayout(root_cols);
}

void FlightLogWidget::updateNamespace(const std::string& ns)
{
  recorder_->updateNamespace(ns);
}
}  // namespace log
}  // namespace gui
