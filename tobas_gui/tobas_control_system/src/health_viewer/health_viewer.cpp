// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/health_viewer/health_viewer.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
HealthViewerWidget::HealthViewerWidget(const RosQtBridge& bridge)
{
  rt_compliance_status_ = new StatusWidget("Realtime Compliance");
  battery_voltage_status_ = new StatusWidget("Battery Voltage");
  cpu_temp_status_ = new StatusWidget("CPU Temperature");
  radio_link_status_ = new StatusWidget("Radio Link");
  rotor_links_status_ = new StatusWidget("Rotor Links");
  level_atti_status_ = new StatusWidget("Level Attitude");
  gnss_fix_status_ = new StatusWidget("GNSS Fix");
  pos_stability_status_ = new StatusWidget("Position Stability");
  hor_pos_accuracy_status_ = new StatusWidget("Horizontal Position Accuracy");
  ver_pos_accuracy_status_ = new StatusWidget("Vertical Position Accuracy");
  atti_accuracy_status_ = new StatusWidget("Attitude Accuracy");
  head_accuracy_status_ = new StatusWidget("Heading Accuracy");
  mag_offset_status_ = new StatusWidget("Mag Offset");
  mag_alignment_status_ = new StatusWidget("Mag Alignment");
  vibration_level_status_ = new StatusWidget("Vibration Level");
  user_defined_status_ = new StatusWidget("User-Defined Condition");

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(rt_compliance_status_);
  rows->addWidget(battery_voltage_status_);
  rows->addWidget(cpu_temp_status_);
  rows->addWidget(radio_link_status_);
  rows->addWidget(rotor_links_status_);
  rows->addWidget(level_atti_status_);
  rows->addWidget(gnss_fix_status_);
  rows->addWidget(pos_stability_status_);
  rows->addWidget(hor_pos_accuracy_status_);
  rows->addWidget(ver_pos_accuracy_status_);
  rows->addWidget(atti_accuracy_status_);
  rows->addWidget(head_accuracy_status_);
  rows->addWidget(mag_offset_status_);
  rows->addWidget(mag_alignment_status_);
  rows->addWidget(vibration_level_status_);
  rows->addWidget(user_defined_status_);
  rows->addStretch();
  setLayout(rows);

  // Connection
  connect(&bridge, &RosQtBridge::vehicleHealthReceived, this, &self::healthCb, Qt::QueuedConnection);
}

void HealthViewerWidget::reset()
{
  rt_compliance_status_->reset();
  battery_voltage_status_->reset();
  cpu_temp_status_->reset();
  radio_link_status_->reset();
  rotor_links_status_->reset();
  level_atti_status_->reset();
  gnss_fix_status_->reset();
  pos_stability_status_->reset();
  hor_pos_accuracy_status_->reset();
  ver_pos_accuracy_status_->reset();
  atti_accuracy_status_->reset();
  head_accuracy_status_->reset();
  mag_offset_status_->reset();
  mag_alignment_status_->reset();
  vibration_level_status_->reset();
  user_defined_status_->reset();
}

void HealthViewerWidget::healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health)
{
  rt_compliance_status_->setStatus(health->realtime_compliance);
  battery_voltage_status_->setStatus(health->battery_voltage);
  cpu_temp_status_->setStatus(health->cpu_temperature);
  radio_link_status_->setStatus(health->radio_link);
  rotor_links_status_->setStatus(health->rotor_links);
  level_atti_status_->setStatus(health->attitude_level);
  gnss_fix_status_->setStatus(health->gnss_fix);
  pos_stability_status_->setStatus(health->position_stability);
  hor_pos_accuracy_status_->setStatus(health->horizontal_position_accuracy);
  ver_pos_accuracy_status_->setStatus(health->vertical_position_accuracy);
  atti_accuracy_status_->setStatus(health->attitude_accuracy);
  head_accuracy_status_->setStatus(health->heading_accuracy);
  mag_offset_status_->setStatus(health->mag_offset);
  mag_alignment_status_->setStatus(health->mag_alignment);
  vibration_level_status_->setStatus(health->vibration_level);
  user_defined_status_->setStatus(health->user_defined_condition);
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
