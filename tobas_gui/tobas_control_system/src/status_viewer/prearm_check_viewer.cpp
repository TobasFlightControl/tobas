#include "tobas_control_system/status_viewer/prearm_check_viewer.hpp"

#include <QVBoxLayout>

namespace gui
{
namespace ctrl
{
PreArmCheckViewerWidget::PreArmCheckViewerWidget(const RosQtBridge& bridge)
{
  node_connection_status_ = new StatusWidget("Node Connection");
  battery_voltage_status_ = new StatusWidget("Battery Voltage");
  cpu_temp_status_ = new StatusWidget("CPU Temperature");
  rotor_comm_status_ = new StatusWidget("Rotors Communication");
  level_atti_status_ = new StatusWidget("Level Attitude");
  pos_stability_status_ = new StatusWidget("Position Stability");
  pos_accuracy_status_ = new StatusWidget("Position Accuracy");
  vel_accuracy_status_ = new StatusWidget("Velocity Accuracy");
  atti_accuracy_status_ = new StatusWidget("Attitude Accuracy");
  head_accuracy_status_ = new StatusWidget("Heading Accuracy");
  ready_arm_status_ = new StatusWidget("Ready to Arm");

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(node_connection_status_);
  rows->addWidget(battery_voltage_status_);
  rows->addWidget(cpu_temp_status_);
  rows->addWidget(rotor_comm_status_);
  rows->addWidget(level_atti_status_);
  rows->addWidget(pos_stability_status_);
  rows->addWidget(pos_accuracy_status_);
  rows->addWidget(vel_accuracy_status_);
  rows->addWidget(atti_accuracy_status_);
  rows->addWidget(head_accuracy_status_);
  rows->addWidget(ready_arm_status_);
  rows->addStretch();
  setLayout(rows);

  // Connection
  connect(&bridge, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::preArmCheckReceived, this, &self::preArmCheckCb, Qt::QueuedConnection);
}

void PreArmCheckViewerWidget::reset()
{
  node_connection_status_->reset();
  battery_voltage_status_->reset();
  cpu_temp_status_->reset();
  rotor_comm_status_->reset();
  level_atti_status_->reset();
  pos_stability_status_->reset();
  pos_accuracy_status_->reset();
  vel_accuracy_status_->reset();
  atti_accuracy_status_->reset();
  head_accuracy_status_->reset();
  ready_arm_status_->reset();
}

void PreArmCheckViewerWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;

  if (arming->data) {
    reset();
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }
}

void PreArmCheckViewerWidget::preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check)
{
  if (!arming_) {
    return;
  }

  if (arming_->data) {
    return;
  }

  node_connection_status_->setStatus(prearm_check->node_connection_unstable);
  battery_voltage_status_->setStatus(prearm_check->battery_voltage_too_low);
  cpu_temp_status_->setStatus(prearm_check->cpu_temperature_too_high);
  rotor_comm_status_->setStatus(prearm_check->rotor_communication_error);
  level_atti_status_->setStatus(prearm_check->attitude_too_steep);
  pos_stability_status_->setStatus(prearm_check->position_unstable);
  pos_accuracy_status_->setStatus(prearm_check->position_inaccurate);
  vel_accuracy_status_->setStatus(prearm_check->velocity_inaccurate);
  atti_accuracy_status_->setStatus(prearm_check->attitude_inaccurate);
  head_accuracy_status_->setStatus(prearm_check->heading_inaccurate);
  ready_arm_status_->setStatus(prearm_check->ok);
}
}  // namespace ctrl
}  // namespace gui
