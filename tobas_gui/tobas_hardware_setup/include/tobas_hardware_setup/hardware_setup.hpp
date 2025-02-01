#pragma once

#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./network_setting/widget.hpp"
#include "./accel_calibration/widget.hpp"
#include "./mag_calibration/widget.hpp"
#include "./rcin_calibration/widget.hpp"
#include "./rotor_test/rotor_test.hpp"
#include "./joint_test/joint_test.hpp"

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

public:
  explicit HardwareSetupWidget(rclcpp::Node::SharedPtr node, const kdl::Tree& tree, const tobas::Drone& drone);

  void updateInternalDataStructures();

private:
  const tobas::Drone& drone_;

  qt::VerticalTabWidget* tabs_;

  NetworkSettingWidget* network_setting_;
  AccelCalibrationWidget* accel_calib_;
  MagCalibrationWidget* mag_calib_;
  RCInputCalibrationWidget* rcin_calib_;
  RotorTestWidget* rotor_test_;
  JointTestWidget* joint_test_;
};
}  // namespace hardware_setup
}  // namespace gui
