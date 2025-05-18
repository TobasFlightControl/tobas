#pragma once

#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp>
#include <tobas_qt_tools/widgets/position_bar_widget.hpp>
#include <tobas_ros2_tools/register.hpp>

#include <tobas_msgs/msg/battery.hpp>

namespace gui
{
namespace gcs
{
class BatteryViewerWidget : public QWidget
{
  Q_OBJECT

  using self = BatteryViewerWidget;
  using super = QWidget;

  static constexpr int kLabelPSize = 12;
  static constexpr int kBarHeight = 40;

Q_SIGNALS:
  void batteryReceived(double voltage, double current);

public:
  explicit BatteryViewerWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone);

  void reset();
  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const tobas::Drone& drone_;
  tobas::ElectricPropulsionSystemConfig::ConstSharedPtr eprop_;

  qt::HPositionBarWidget* voltage_;
  qt::HPositionBarWidget* current_;

  ros2::SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;

  void updateVoltage(const double& voltage);
  void updateCurrent(const double& current);

  void batteryCbRos(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void batteryCbQt(double voltage, double current);
};
}  // namespace gcs
}  // namespace gui
