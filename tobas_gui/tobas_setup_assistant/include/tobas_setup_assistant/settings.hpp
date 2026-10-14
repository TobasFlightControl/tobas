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

namespace tobas::gui::sa
{
class SettingsWidget : public QWidget
{
  Q_OBJECT

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

  explicit SettingsWidget(const uadf::Model& _uadf, const kdl::Tree& _tree, Signals& _sig);

  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  bool load(const YAML::Node& _node);

  void setFrameType(FrameType _type);

private:
  const uadf::Model& uadf_;

  SettingsNavigationWidget* navigation_;
  qt::StackedWidget* stack_;

  int getIndex(BaseSettingWidget* _page) const;
  void addPage(BaseSettingWidget* _page);
  void setCurrentPage(int _idx);
  void setCurrentPage(BaseSettingWidget* _page);
  void setPageEnabled(int _idx, bool _enabled, const QString& _disabled_reason = {});
  void setPageEnabled(BaseSettingWidget* _page, bool _enabled, const QString& _disabled_reason = {});
};
}  // namespace tobas::gui::sa
