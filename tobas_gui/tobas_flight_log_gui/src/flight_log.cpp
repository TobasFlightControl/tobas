#include <QHBoxLayout>

#include "tobas_flight_log_gui/flight_log.hpp"

namespace gui
{
namespace log
{
FlightLogWidget::FlightLogWidget(rclcpp::Node::SharedPtr node)
{
  recorder_ = new FlightLogRecorderWidget(node);
  reader_ = new FlightLogReaderWidget(node);

  const auto cols = new QHBoxLayout();
  cols->addWidget(recorder_, 1);
  cols->addWidget(reader_, 1);

  setLayout(cols);
}

void FlightLogWidget::updateNamespace(const std::string& ns)
{
  recorder_->updateNamespace(ns);
}
}  // namespace log
}  // namespace gui
