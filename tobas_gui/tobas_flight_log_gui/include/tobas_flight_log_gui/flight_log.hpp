#pragma once

#include "./recorder.hpp"
#include "./logs_fc/logs_widget.hpp"
#include "./logs_gcs/logs_widget.hpp"

namespace gui
{
namespace log
{
class FlightLogWidget : public QWidget
{
  Q_OBJECT

  using self = FlightLogWidget;
  using super = QWidget;

public:
  explicit FlightLogWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

private:
  FlightLogRecorderWidget* recorder_;
  FlightLogsWidgetFC* logs_fc_;
  FlightLogsWidgetGCS* logs_gcs_;
};
}  // namespace log
}  // namespace gui
