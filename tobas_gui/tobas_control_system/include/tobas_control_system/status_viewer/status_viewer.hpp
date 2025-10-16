#pragma once

#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "./status.hpp"

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
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

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

  StatusWidget* ready_arm_status_;
  StatusWidget* armed_status_;

private Q_SLOTS:
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health);
};
}  // namespace ctrl
}  // namespace gui
