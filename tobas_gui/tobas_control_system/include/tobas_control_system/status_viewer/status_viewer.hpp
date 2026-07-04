// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "./status.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
class StatusViewerWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = StatusViewerWidget;
  using super = qt::ScrollArea;

public:
  explicit StatusViewerWidget(const RosQtBridge& bridge);

  void reset();

private:
  StatusWidget* rt_compliance_status_;
  StatusWidget* battery_voltage_status_;
  StatusWidget* cpu_temp_status_;
  StatusWidget* radio_link_status_;
  StatusWidget* rotor_links_status_;
  StatusWidget* level_atti_status_;
  StatusWidget* pos_stability_status_;
  StatusWidget* pos_accuracy_status_;
  StatusWidget* vel_accuracy_status_;
  StatusWidget* atti_accuracy_status_;
  StatusWidget* head_accuracy_status_;
  StatusWidget* mag_offset_status_;
  StatusWidget* mag_alignment_status_;
  StatusWidget* vibration_level_status_;
  StatusWidget* user_defined_status_;

private Q_SLOTS:
  void healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health);
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
