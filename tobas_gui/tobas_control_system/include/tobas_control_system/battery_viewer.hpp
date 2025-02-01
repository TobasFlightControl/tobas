#pragma once

#include <tobas_ros2_tools/register.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/position_bar_widget.hpp>
#include <tobas_msgs/msg/battery.hpp>

namespace gui
{
namespace control_system
{
class BatteryViewerWidget : public QWidget
{
  Q_OBJECT

  using self = BatteryViewerWidget;
  using super = QWidget;

  static constexpr int kLabelPSize = 12;
  static constexpr int kBarHeight = 40;

public:
  explicit BatteryViewerWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone);

  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const tobas::Drone& drone_;

  qt::HPositionBarWidget* voltage_;
  qt::HPositionBarWidget* current_;

  ros2::SubscriberPtr<tobas_msgs::msg::Battery> batt_sub_;

  void battCb(const tobas_msgs::msg::Battery::ConstSharedPtr& batt);
};
}  // namespace control_system
}  // namespace gui
