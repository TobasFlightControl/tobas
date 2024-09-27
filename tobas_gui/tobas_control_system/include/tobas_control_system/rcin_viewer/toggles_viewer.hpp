#pragma once

#include <tobas_ros2_tools/register.hpp>
#include <tobas_qt_tools/widgets/circle_widget.hpp>
#include <tobas_msgs/msg/rc_input.hpp>

namespace gui
{
namespace control_system
{
namespace rcin
{
class TogglesViewer : public QWidget
{
  Q_OBJECT

  using self = TogglesViewer;
  using super = QWidget;

public:
  explicit TogglesViewer(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  qt::CircleWidget* program_mode_;
  qt::CircleWidget* stabilize_mode_;
  qt::CircleWidget* acrobat_mode_;

  qt::CircleWidget* estop_;
  qt::CircleWidget* gpsw_;

  ros2::SubscriberPtr<tobas_msgs::msg::RCInput> rcin_sub_;

  void reset();

  void rcInputCb(const tobas_msgs::msg::RCInput::ConstSharedPtr& rcin);
};
}  // namespace rcin
}  // namespace control_system
}  // namespace gui
