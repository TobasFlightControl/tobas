#pragma once

#include <QLineEdit>
#include <QPushButton>
#include <QLCDNumber>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>
#include <tobas_qt_tools/widgets/position_bar_widget.hpp>
#include <tobas_qt_tools/widgets/framed_label.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>

#include <tobas_msgs/msg/rosbag_state.hpp>
#include <tobas_msgs/srv/bag_record_stop.hpp>

#include "./start_thread.hpp"
#include "./stop_thread.hpp"

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

public:
  explicit FlightLogRecorderWidget(rclcpp::Node::SharedPtr node);

  void reset();
  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  QLineEdit* log_name_;
  qt::ToggleButton* start_stop_button_;
  QLCDNumber* duration_;
  qt::HPositionBarWidget* file_size_;
  qt::FramedLabel* message_count_;

  RecordStartThread start_thread_;
  RecordStopThread stop_thread_;

  qt::WaitSpinnerWidget spinner_;

  tobas_msgs::msg::RosbagState::ConstSharedPtr rosbag_state_;

  ros2::SubscriberPtr<tobas_msgs::msg::RosbagState> rosbag_state_sub_;

  void clearRosbagStateViewerWidgets();

  void rosbagStateCb(const tobas_msgs::msg::RosbagState::ConstSharedPtr& rosbag_state);

private Q_SLOTS:
  void onStartRequested();
  void onStopRequested();

  void onStartThreadFinished(bool success, const QString& message);
  void onStopThreadFinished(bool success, const QString& message);
};
}  // namespace log
}  // namespace gui
