// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./log_viewer/log_viewer.hpp"
#include "./logs_fc/logs_widget.hpp"
#include "./logs_gcs/logs_widget.hpp"
#include "./recorder/recorder.hpp"

namespace tobas
{
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
  explicit FlightLogWidget(const RosQtBridge& bridge);

  void reset();
  void onProjectLoaded();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);

private:
  FlightLogRecorderWidget* recorder_;
  FlightLogsWidgetFC* logs_fc_;
  FlightLogsWidgetGCS* logs_gcs_;
  FlightLogViewerWidget* log_viewer_;

private Q_SLOTS:
  void onRecordFinished(const QString& log_name, bool is_real);
  void onLogDownloaded(const QString& log_name);
  void onLogSelected(const QString& log_name);
  void onLogDeselected();
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
