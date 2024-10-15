#pragma once

#include <tobas_ros2_tools/register.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/position_bar_widget.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>

namespace gui
{
namespace control_system
{
class BatteryCPUViewerWidget : public QWidget
{
  Q_OBJECT

  using self = BatteryCPUViewerWidget;
  using super = QWidget;

  static constexpr int kLabelPSize = 12;
  static constexpr int kBarHeight = 40;

  static constexpr double kMinCPUTemp = 0.;    // [degC]
  static constexpr double kMaxCPUTemp = 90.;   // [degC]
  static constexpr double kMinCPULoad = 0.;    // [%]
  static constexpr double kMaxCPULoad = 100.;  // [%]

public:
  explicit BatteryCPUViewerWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone);

  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const tobas::Drone& drone_;

  qt::HPositionBarWidget* batt_voltage_;
  qt::HPositionBarWidget* cpu_temp_;
  qt::HPositionBarWidget* cpu_load_;

  ros2::SubscriberPtr<tobas_msgs::msg::Battery> batt_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Cpu> cpu_sub_;

  void battCb(const tobas_msgs::msg::Battery::ConstSharedPtr& batt);
  void cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu);
};
}  // namespace control_system
}  // namespace gui
