#pragma once

#include "./log_viewer/log_viewer.hpp"
#include "./logs_fc/logs_widget.hpp"
#include "./logs_gcs/logs_widget.hpp"
#include "./recorder/recorder.hpp"

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
  explicit FlightLogWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge);

  void reset();
  void updateNamespace(const std::string& ns);

private:
  FlightLogRecorderWidget* recorder_;
  FlightLogsWidgetFC* logs_fc_;
  FlightLogsWidgetGCS* logs_gcs_;
  FlightLogViewerWidget* log_viewer_;

private Q_SLOTS:
  void onLogDownloaded(const QString& log_name);
  void onLogSelected(const QString& log_name);
};
}  // namespace log
}  // namespace gui
