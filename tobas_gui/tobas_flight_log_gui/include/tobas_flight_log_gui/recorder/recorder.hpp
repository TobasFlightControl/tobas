// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLCDNumber>

#include <tobas_qt_tools/widgets/framed_label.hpp>
#include <tobas_qt_tools/widgets/history_line_edit.hpp>
#include <tobas_qt_tools/widgets/position_bar_widget.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "./start_thread.hpp"
#include "./stop_thread.hpp"

namespace tobas::gui::log
{
class FlightLogRecorderWidget : public QWidget
{
  Q_OBJECT

  using self = FlightLogRecorderWidget;

Q_SIGNALS:
  void recordFinished(const QString& _log_name, bool _is_real);

public:
  explicit FlightLogRecorderWidget(const rqt::RosQtBridge& _bridge);

  void reset();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns);
  void clearRosInterfaces();

private:
  qt::HistoryLineEdit* log_name_;
  qt::ToggleButton* start_stop_button_;
  QLCDNumber* duration_;
  qt::HPositionBarWidget* file_size_;
  qt::FramedLabel* message_count_;

  RecordStartThread start_thread_;
  RecordStopThread stop_thread_;

  qt::WaitSpinnerWidget spinner_;

  bool rosbag_state_received_ = false;

  void updateStartStopButtonAvailability();
  void clearRosbagStateViewerWidgets();

private Q_SLOTS:
  void onLogNameChanged();
  void onStartRequested();
  void onStopRequested();

  void rosbagStateCb(const tobas_msgs::msg::RosbagState::ConstSharedPtr& _msg);
};
}  // namespace tobas::gui::log
