#pragma once

#include <tobas_ros2_tools/register.hpp>
#include <tobas_qt_tools/widgets/position_bar_widget.hpp>
#include <tobas_msgs/msg/rc_input.hpp>

namespace gui
{
namespace control_system
{
class RCInputThrottlesViewer : public QWidget
{
  Q_OBJECT

  using self = RCInputThrottlesViewer;
  using super = QWidget;

  static constexpr int kRangeSideShort = 30;

public:
  explicit RCInputThrottlesViewer(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  qt::HPositionBarWidget* roll_range_;
  qt::VPositionBarWidget* pitch_range_;
  qt::HPositionBarWidget* yaw_range_;
  qt::VPositionBarWidget* throt_range_;

  ros2::SubscriberPtr<tobas_msgs::msg::RCInput> rcin_sub_;

  void rcInputCb(const tobas_msgs::msg::RCInput::ConstSharedPtr& rcin);
};
}  // namespace control_system
}  // namespace gui
