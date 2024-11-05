#pragma once

#include <QHBoxLayout>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>

#include "./speedmeter.hpp"

namespace gui
{
namespace control_system
{
class RotorsViewerWiddget : public qt::ScrollArea
{
  Q_OBJECT

  using self = RotorsViewerWiddget;
  using super = qt::ScrollArea;

public:
  explicit RotorsViewerWiddget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone);

  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const tobas::Drone& drone_;

  std::vector<SpeedmeterWidget*> meters_;
  QHBoxLayout* cols_;

  ros2::SubscriberPtr<tobas_msgs::msg::RotorSpeedArray> speeds_sub_;

  void speedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& speeds);

  static QString bottomText(int rpm);
};
}  // namespace control_system
}  // namespace gui
