#pragma once

#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./setting_tabs/author_information.hpp"
#include "./setting_tabs/controller/controller.hpp"
#include "./setting_tabs/extra_joints.hpp"
#include "./setting_tabs/fixed_wing/fixed_wing.hpp"
#include "./setting_tabs/hardware/hardware.hpp"
#include "./setting_tabs/observer.hpp"
#include "./setting_tabs/pre_arm_check.hpp"
#include "./setting_tabs/propulsion_system/propulsion_system.hpp"
#include "./setting_tabs/rc_input.hpp"
#include "./setting_tabs/simulation.hpp"

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
  const uadf::Model& uadf_;

  propulsion::PropulsionSystemWidget* propulsion_system;
  fw::FixedWingWidget* fixed_wing;
  ExtraJointsWidget* extra_joints;
  RcInputWidget* rc_input;
  ctrl::ControllerWidget* controller;
  ObserverWidget* observer;
  hw::HardwareWidget* hardware;
  PreArmCheckWidget* pre_arm_check;
  SimulationWidget* simulation;
  AuthorInformationWidget* author_info;

  explicit SettingsWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf, const kdl::Tree& tree, Signals& sig);

  void updateInternalDataStructures();

  bool isValid();

  YAML::Node dump() const;
  bool load(const YAML::Node& node);
};
}  // namespace sa
}  // namespace gui
