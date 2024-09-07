#pragma once

#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./robot_info.hpp"

#include "./setting_tabs/battery/battery.hpp"
#include "./setting_tabs/propulsion_system/propulsion_system.hpp"
#include "./setting_tabs/fixed_wing/fixed_wing.hpp"
#include "./setting_tabs/servo_joint/servo_joint.hpp"
#include "./setting_tabs/imu.hpp"
#include "./setting_tabs/magnetometer.hpp"
#include "./setting_tabs/barometer.hpp"
#include "./setting_tabs/gps.hpp"
#include "./setting_tabs/controller/controller.hpp"
#include "./setting_tabs/observer/observer.hpp"
#include "./setting_tabs/hardware/hardware.hpp"
#include "./setting_tabs/simulation.hpp"
#include "./setting_tabs/author_information.hpp"
#include "./setting_tabs/ros_package.hpp"

namespace gui
{
namespace setup_assistant
{
class SettingsWidget : public qt::VerticalTabWidget
{
  Q_OBJECT

  using self = SettingsWidget;
  using super = qt::VerticalTabWidget;

  static constexpr int kTabHeight = 30;  // 30以上無いと何故かTabBarの文字が横に見切れてしまう
  static constexpr int kTabWidth = 70;
  static constexpr int kSettingsMinHeight = 300;

public:
  BatteryWidget* battery;
  propulsion_system::PropulsionSystemWidget* propulsion_system;
  fixed_wing::FixedWingWidget* fixed_wing;
  servo_joint::ServoJointsWidget* servo_joints;
  IMUWidget* imu;
  MagnetometerWidget* magnetometer;
  BarometerWidget* barometer;
  GPSWidget* gps;
  ControllerWidget* controller;
  ObserverWidget* observer;
  HardwareWidget* hardware;
  SimulationWidget* simulation;
  AuthorInformationWidget* author_info;
  ROSPackageWidget* ros_package;

  explicit SettingsWidget(rclcpp::Node::SharedPtr node, RobotInfo& robot);

  void updateInternalDataStructures();

  bool isValid();

  YAML::Node dump();
  bool load(const YAML::Node& node);

private Q_SLOTS:
  void onCurrentChanged(int index);
};
}  // namespace setup_assistant
}  // namespace gui
