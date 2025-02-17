#pragma once

#include <tobas_ros2_tools/register.hpp>
#include <tobas_qt_tools/widgets/circle_widget.hpp>
#include <tobas_qt_tools/widgets/toggle_switch.hpp>
#include <tobas_msgs/msg/rc_input.hpp>

namespace gui
{
namespace gcs
{
namespace rcin
{
class TogglesViewer : public QWidget
{
  Q_OBJECT

  using self = TogglesViewer;
  using super = QWidget;

  static constexpr auto kOffColor = Qt::gray;
  static constexpr auto kOnColorEnable = Qt::green;
  static constexpr auto kOnColorDisable = Qt::darkGray;

public:
  explicit TogglesViewer(rclcpp::Node::SharedPtr node);

  void reset();
  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  qt::CircleWidget* acrobat_mode_;
  qt::CircleWidget* stabilize_mode_;
  qt::CircleWidget* loiter_mode_;

  qt::ToggleSwitch* enable_;
  qt::ToggleSwitch* gpsw_;

  ros2::SubscriberPtr<tobas_msgs::msg::RCInput> rcin_sub_;

  void rcInputCb(const tobas_msgs::msg::RCInput::ConstSharedPtr& rcin);
};
}  // namespace rcin
}  // namespace gcs
}  // namespace gui
