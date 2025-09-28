#include "tobas_control_system/status_viewer/status_viewer.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/label.hpp>

namespace gui
{
namespace ctrl
{
StatusViewerWidget::StatusViewerWidget(const RosQtBridge& bridge)
{
  rt_compliance_status_ = new StatusWidget("Realtime Compliance");
  battery_voltage_status_ = new StatusWidget("Battery Voltage");
  cpu_temp_status_ = new StatusWidget("CPU Temperature");
  rotor_comm_status_ = new StatusWidget("Rotors Communication");
  level_atti_status_ = new StatusWidget("Level Attitude");
  pos_stability_status_ = new StatusWidget("Position Stability");
  pos_accuracy_status_ = new StatusWidget("Position Accuracy");
  vel_accuracy_status_ = new StatusWidget("Velocity Accuracy");
  atti_accuracy_status_ = new StatusWidget("Attitude Accuracy");
  head_accuracy_status_ = new StatusWidget("Heading Accuracy");
  mag_offset_status_ = new StatusWidget("Mag Offset");
  mag_alignment_status_ = new StatusWidget("Mag Alignment");
  vibration_level_status_ = new StatusWidget("Vibration Level");
  ready_arm_status_ = new StatusWidget("Ready to Arm");
  armed_status_ = new StatusWidget("Armed");

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(rt_compliance_status_);
  rows->addWidget(battery_voltage_status_);
  rows->addWidget(cpu_temp_status_);
  rows->addWidget(rotor_comm_status_);
  rows->addWidget(level_atti_status_);
  rows->addWidget(pos_stability_status_);
  rows->addWidget(pos_accuracy_status_);
  rows->addWidget(vel_accuracy_status_);
  rows->addWidget(atti_accuracy_status_);
  rows->addWidget(head_accuracy_status_);
  rows->addWidget(mag_offset_status_);
  rows->addWidget(mag_alignment_status_);
  rows->addWidget(vibration_level_status_);
  rows->addWidget(ready_arm_status_);
  rows->addWidget(armed_status_);
  rows->addStretch();
  setLayout(rows);

  // Connection
  connect(&bridge, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::vehicleHealthReceived, this, &self::healthCb, Qt::QueuedConnection);
}

void StatusViewerWidget::reset()
{
  rt_compliance_status_->reset();
  battery_voltage_status_->reset();
  cpu_temp_status_->reset();
  rotor_comm_status_->reset();
  level_atti_status_->reset();
  pos_stability_status_->reset();
  pos_accuracy_status_->reset();
  vel_accuracy_status_->reset();
  atti_accuracy_status_->reset();
  head_accuracy_status_->reset();
  mag_offset_status_->reset();
  mag_alignment_status_->reset();
  vibration_level_status_->reset();
  ready_arm_status_->reset();
  armed_status_->reset();
}

void StatusViewerWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  armed_status_->setStatus(arming->data);

  arming_ = arming;
}

void StatusViewerWidget::healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health)
{
  if (!arming_) {
    return;
  }

  rt_compliance_status_->setStatus(health->rt_violation);
  battery_voltage_status_->setStatus(health->battery_voltage_too_low);
  cpu_temp_status_->setStatus(health->cpu_temperature_too_high);
  rotor_comm_status_->setStatus(health->rotor_communication_error);
  level_atti_status_->setStatus(health->attitude_too_steep);
  pos_stability_status_->setStatus(health->position_unstable);
  pos_accuracy_status_->setStatus(health->position_inaccurate);
  vel_accuracy_status_->setStatus(health->velocity_inaccurate);
  atti_accuracy_status_->setStatus(health->attitude_inaccurate);
  head_accuracy_status_->setStatus(health->heading_inaccurate);
  mag_offset_status_->setStatus(health->mag_offset_too_large);
  mag_alignment_status_->setStatus(health->mag_misalignment);
  vibration_level_status_->setStatus(health->vibration_too_high);

  if (arming_->data) {
    ready_arm_status_->setStatus(StatusWidget::IGNORED);
  }
  else {
    ready_arm_status_->setStatus(health->ok);
  }
}
}  // namespace ctrl
}  // namespace gui
