#pragma once

#include "./flight_log_recorder.hpp"
#include "./flight_log_reader.hpp"

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
  FlightLogReaderWidget* reader_;
};
}  // namespace log
}  // namespace gui
