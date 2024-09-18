#pragma once

#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./accel_calibration/widget.hpp"

namespace gui
{
namespace hardware_setup
{
class HardwareSetupWidget : public QWidget
{
  Q_OBJECT

  using self = HardwareSetupWidget;
  using super = QWidget;

  static constexpr int kTabHeight = 35;  // これ以上無いと何故かTabBarの文字が横に見切れてしまう
  static constexpr int kTabWidth = 70;
  static constexpr int kMinHeight = 300;

public:
  explicit HardwareSetupWidget(
    rclcpp::Node::SharedPtr node,
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_node_if);

private:
  qt::VerticalTabWidget* tabs_;

  AccelCalibrationWidget* accel_calib_;
};
}  // namespace hardware_setup
}  // namespace gui
