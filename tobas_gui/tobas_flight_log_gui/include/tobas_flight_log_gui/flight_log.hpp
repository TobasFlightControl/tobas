// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./log_viewer/log_viewer.hpp"
#include "./logs_fc/logs_widget.hpp"
#include "./logs_gcs/logs_widget.hpp"
#include "./recorder/recorder.hpp"

namespace tobas::gui::log
{
class FlightLogWidget : public QWidget
{
  Q_OBJECT

  using self = FlightLogWidget;

public:
  explicit FlightLogWidget(const rqt::RosQtBridge& _bridge);

  void reset();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns);
  void clearRosInterfaces();

private:
  FlightLogRecorderWidget* recorder_;
  FlightLogsWidgetFC* logs_fc_;
  FlightLogsWidgetGCS* logs_gcs_;
  FlightLogViewerWidget* log_viewer_;

private Q_SLOTS:
  void onRecordFinished(const QString& _log_name, bool _is_real);
  void onLogDownloaded(const QString& _log_name);
  void onLogSelected(const QString& _log_name);
  void onLogDeselected();
};
}  // namespace tobas::gui::log
