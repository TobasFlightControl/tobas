// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "./setting_tabs/author_information.hpp"
#include "./setting_tabs/controller/controller.hpp"
#include "./setting_tabs/extra_joints.hpp"
#include "./setting_tabs/failsafe.hpp"
#include "./setting_tabs/fixed_wing/fixed_wing.hpp"
#include "./setting_tabs/hardware/hardware.hpp"
#include "./setting_tabs/mission_executor/mission_executor.hpp"
#include "./setting_tabs/network/network.hpp"
#include "./setting_tabs/observer.hpp"
#include "./setting_tabs/propulsion_system/propulsion_system.hpp"
#include "./setting_tabs/rc_input.hpp"
#include "./settings_navigation.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class SettingsWidget : public QWidget
{
  Q_OBJECT

  using self = SettingsWidget;
  using super = QWidget;

public:
  propulsion::PropulsionSystemWidget* propulsion_system;
  fw::FixedWingWidget* fixed_wing;
  hw::HardwareWidget* hardware;
  network::NetworkWidget* network;
  ObserverWidget* observer;
  ctrl::ControllerWidget* controller;
  mission::MissionExecutorWidget* mission;
  RcInputWidget* rc_input;
  ExtraJointsWidget* extra_joints;
  FailsafeWidget* failsafe;
  AuthorInformationWidget* author_info;

  explicit SettingsWidget(const uadf::Model& uadf, const kdl::Tree& tree, Signals& sig);

  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  bool load(const YAML::Node& node);

  void setFrameType(FrameType type);

private:
  const uadf::Model& uadf_;

  SettingsNavigationWidget* navigation_;
  qt::StackedWidget* stack_;

  int getIndex(BaseSettingWidget* page) const;
  void addPage(BaseSettingWidget* page);
  void setCurrentPage(int idx);
  void setCurrentPage(BaseSettingWidget* page);
  void setPageEnabled(int idx, bool enabled, const QString& disabled_reason = {});
  void setPageEnabled(BaseSettingWidget* page, bool enabled, const QString& disabled_reason = {});
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
