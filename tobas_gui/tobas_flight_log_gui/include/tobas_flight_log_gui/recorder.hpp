#pragma once

#include <QLineEdit>
#include <QPushButton>
#include <QLCDNumber>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_qt_tools/widgets/position_bar_widget.hpp>
#include <tobas_qt_tools/widgets/framed_label.hpp>
#include <tobas_msgs/msg/rosbag_state.hpp>

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

  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;
  std::string ns_;

  QLineEdit* log_name_;
  QPushButton* start_button_;
  QPushButton* stop_button_;
  QLCDNumber* duration_;
  qt::HPositionBarWidget* file_size_;
  qt::FramedLabel* message_count_;

  ros2::SubscriberPtr<tobas_msgs::msg::RosbagState> rosbag_state_sub_;

  void rosbagStateCb(const tobas_msgs::msg::RosbagState::ConstSharedPtr& rosbag_state);

private Q_SLOTS:
  void onStartButtonClicked();
  void onStopButtonClicked();
};
}  // namespace log
}  // namespace gui
