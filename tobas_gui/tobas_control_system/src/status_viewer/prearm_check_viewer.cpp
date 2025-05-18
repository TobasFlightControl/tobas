#include "tobas_control_system/status_viewer/prearm_check_viewer.hpp"

#include <QVBoxLayout>

#include <tobas_constants/constants.hpp>
#include <tobas_path_tools/join.hpp>

namespace gui
{
namespace gcs
{
PreArmCheckViewerWidget::PreArmCheckViewerWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  node_connection_status_ = new StatusWidget("Node Connection");
  battery_voltage_status_ = new StatusWidget("Battery Voltage");
  cpu_temp_status_ = new StatusWidget("CPU Temperature");
  rotor_comm_status_ = new StatusWidget("Rotors Communication");
  attitude_level_status_ = new StatusWidget("Attitude Horizontal");
  pos_stability_status_ = new StatusWidget("Position Stable");
  pos_accuracy_status_ = new StatusWidget("Position Estimation Accurate");
  vel_accuracy_status_ = new StatusWidget("Velocity Estimation Accurate");
  atti_accuracy_status_ = new StatusWidget("Attitude Estimation Accurate");
  head_accuracy_status_ = new StatusWidget("Heading Estimation Accurate");
  ready_status_ = new StatusWidget("Ready to Arm");

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(node_connection_status_);
  rows->addWidget(battery_voltage_status_);
  rows->addWidget(cpu_temp_status_);
  rows->addWidget(rotor_comm_status_);
  rows->addWidget(attitude_level_status_);
  rows->addWidget(pos_stability_status_);
  rows->addWidget(pos_accuracy_status_);
  rows->addWidget(vel_accuracy_status_);
  rows->addWidget(atti_accuracy_status_);
  rows->addWidget(head_accuracy_status_);
  rows->addWidget(ready_status_);
  rows->addStretch();
  setLayout(rows);

  // Connection
  connect(this, &self::armingReceived, this, &self::armingCbQt, Qt::QueuedConnection);
  connect(this, &self::preArmCheckReceived, this, &self::preArmCheckCbQt, Qt::QueuedConnection);
}

void PreArmCheckViewerWidget::reset()
{
  node_connection_status_->reset();
  battery_voltage_status_->reset();
  cpu_temp_status_->reset();
  rotor_comm_status_->reset();
  attitude_level_status_->reset();
  pos_stability_status_->reset();
  pos_accuracy_status_->reset();
  vel_accuracy_status_->reset();
  atti_accuracy_status_->reset();
  head_accuracy_status_->reset();
  ready_status_->reset();
}

void PreArmCheckViewerWidget::updateNamespace(const std::string& ns)
{
  reset();

  arming_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kArmingTopic), &self::armingCbRos, this);
  prearm_check_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kPreArmCheckTopic), &self::preArmCheckCbRos, this);
}

void PreArmCheckViewerWidget::armingCbRos(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
  Q_EMIT armingReceived(arming->data);
}

void PreArmCheckViewerWidget::preArmCheckCbRos(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check)
{
  Q_EMIT preArmCheckReceived(
    prearm_check->node_connection_unstable,
    prearm_check->battery_voltage_too_low,
    prearm_check->cpu_temperature_too_high,
    prearm_check->rotor_communication_error,
    prearm_check->attitude_too_steep,
    prearm_check->position_unstable,
    prearm_check->position_inaccurate,
    prearm_check->velocity_inaccurate,
    prearm_check->attitude_inaccurate,
    prearm_check->heading_inaccurate,
    prearm_check->ok);
}

void PreArmCheckViewerWidget::armingCbQt(bool arming)
{
  if (arming) {
    reset();
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }
}

void PreArmCheckViewerWidget::preArmCheckCbQt(
  uint8_t node_connection_unstable,
  uint8_t battery_voltage_too_low,
  uint8_t cpu_temperature_too_high,
  uint8_t rotor_communication_error,
  uint8_t attitude_too_steep,
  uint8_t position_unstable,
  uint8_t position_inaccurate,
  uint8_t velocity_inaccurate,
  uint8_t attitude_inaccurate,
  uint8_t heading_inaccurate,
  bool ok)
{
  if (!arming_) {
    return;
  }

  if (arming_->data) {
    return;
  }

  node_connection_status_->setStatus(node_connection_unstable);
  battery_voltage_status_->setStatus(battery_voltage_too_low);
  cpu_temp_status_->setStatus(cpu_temperature_too_high);
  rotor_comm_status_->setStatus(rotor_communication_error);
  attitude_level_status_->setStatus(attitude_too_steep);
  pos_stability_status_->setStatus(position_unstable);
  pos_accuracy_status_->setStatus(position_inaccurate);
  vel_accuracy_status_->setStatus(velocity_inaccurate);
  atti_accuracy_status_->setStatus(attitude_inaccurate);
  head_accuracy_status_->setStatus(heading_inaccurate);
  ready_status_->setStatus(ok);
}
}  // namespace gcs
}  // namespace gui
