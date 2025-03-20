#pragma once

#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./robot_info.hpp"

#include "./setting_tabs/propulsion_system/propulsion_system.hpp"
#include "./setting_tabs/fixed_wing/fixed_wing.hpp"
#include "./setting_tabs/joint_config.hpp"
#include "./setting_tabs/imu.hpp"
#include "./setting_tabs/magnetometer.hpp"
#include "./setting_tabs/barometer.hpp"
#include "./setting_tabs/gnss.hpp"
#include "./setting_tabs/controller/controller.hpp"
#include "./setting_tabs/observer/observer.hpp"
#include "./setting_tabs/hardware/hardware.hpp"
#include "./setting_tabs/pre_arm_check.hpp"
#include "./setting_tabs/simulation.hpp"
#include "./setting_tabs/author_information.hpp"
#include "./setting_tabs/ros_package.hpp"

namespace gui
{
namespace sa
{
class SettingsWidget : public qt::VerticalTabWidget
{
  Q_OBJECT

  using self = SettingsWidget;
  using super = qt::VerticalTabWidget;

  static constexpr int kTabHeight = 30;  // 30以上無いと何故かTabBarの文字が横に見切れてしまう
  static constexpr int kTabWidth = 70;

public:
  propulsion::PropulsionSystemWidget* propulsion_system;
  fixed_wing::FixedWingWidget* fixed_wing;
  JointConfigurationWidget* joint_config;
  IMUWidget* imu;
  MagnetometerWidget* magnetometer;
  BarometerWidget* barometer;
  GNSSWidget* gnss;
  ControllerWidget* controller;
  ObserverWidget* observer;
  HardwareWidget* hardware;
  PreArmCheckWidget* pre_arm_check;
  SimulationWidget* simulation;
  AuthorInformationWidget* author_info;
  ROSPackageWidget* ros_package;

  explicit SettingsWidget(rclcpp::Node::SharedPtr node, RobotInfo& robot, Signals& _signals);

  void updateInternalDataStructures();

  bool isValid();

  YAML::Node dump() const;
  bool load(const YAML::Node& node);

private Q_SLOTS:
  void onCurrentChanged(int index);
};
}  // namespace sa
}  // namespace gui
