#pragma once

#include <QLineEdit>
#include <QPushButton>
#include <std_msgs/msg/bool.hpp>

#include <tobas_ros2_tools/register.hpp>

namespace gui
{
namespace log
{
class FlightLogRecorderWidget : public QWidget
{
  Q_OBJECT

  using self = FlightLogRecorderWidget;
  using super = QWidget;

  static constexpr int kLogNameLabelPSize = 12;
  static constexpr int kButtonWidth = 150;
  static constexpr int kButtonHeight = 60;

public:
  explicit FlightLogRecorderWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;
  std::string ns_;

  QLineEdit* rosbag_name_;
  QPushButton* start_button_;
  QPushButton* stop_button_;

  std_msgs::msg::Bool::ConstSharedPtr arming_;
  ros2::SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);

private Q_SLOTS:
  void onStartButtonClicked();
  void onStopButtonClicked();
};
}  // namespace log
}  // namespace gui
