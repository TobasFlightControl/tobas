#pragma once

#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./network_setting/widget.hpp"
#include "./accel_calibration/widget.hpp"
#include "./mag_calibration/widget.hpp"
#include "./adc_calibration/widget.hpp"
#include "./rcin_calibration/widget.hpp"
#include "./rotor_test/rotor_test.hpp"

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

  NetworkSettingWidget* network_setting_;
  AccelCalibrationWidget* accel_calib_;
  MagCalibrationWidget* mag_calib_;
  ADCCalibrationWidget* adc_calib_;
  RCInputCalibrationWidget* rcin_calib_;
  RotorTestWidget* rotor_test_;
};
}  // namespace hardware_setup
}  // namespace gui
