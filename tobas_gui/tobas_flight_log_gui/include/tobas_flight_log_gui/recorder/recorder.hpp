// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLCDNumber>
#include <QLineEdit>
#include <QPushButton>

#include <tobas_qt_tools/widgets/framed_label.hpp>
#include <tobas_qt_tools/widgets/history_line_edit.hpp>
#include <tobas_qt_tools/widgets/position_bar_widget.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "./start_thread.hpp"
#include "./stop_thread.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class FlightLogRecorderWidget : public QWidget
{
  Q_OBJECT

  using self = FlightLogRecorderWidget;
  using super = QWidget;

  static constexpr int kButtonWidth = 150;
  static constexpr int kButtonHeight = 60;

Q_SIGNALS:
  void recordFinished(const QString& log_name, bool is_real);

public:
  explicit FlightLogRecorderWidget(const RosQtBridge& bridge);

  void reset();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);

private:
  qt::HistoryLineEdit* log_name_;
  qt::ToggleButton* start_stop_button_;
  QLCDNumber* duration_;
  qt::HPositionBarWidget* file_size_;
  qt::FramedLabel* message_count_;

  RecordStartThread start_thread_;
  RecordStopThread stop_thread_;

  qt::WaitSpinnerWidget spinner_;

  tobas_msgs::msg::RosbagState::ConstSharedPtr rosbag_state_;
  bool ros_initialized_ = false;

  void clearRosbagStateViewerWidgets();

private Q_SLOTS:
  void onStartRequested();
  void onStopRequested();

  void rosbagStateCb(const tobas_msgs::msg::RosbagState::ConstSharedPtr& rosbag_state);
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
