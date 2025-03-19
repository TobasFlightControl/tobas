#include <QVBoxLayout>

#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>

#include "tobas_control_system/status_viewer/prearm_check_viewer.hpp"

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
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kArmingTopic), &self::armingCb, this);
  prearm_check_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kPreArmCheckTopic), &self::preArmCheckCb, this);
}

void PreArmCheckViewerWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;

  if (arming->data)
  {
    reset();
    setEnabled(false);
  }
  else
  {
    setEnabled(true);
  }
}

void PreArmCheckViewerWidget::preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check)
{
  if (!arming_)
  {
    reset();
    return;
  }

  if (arming_->data)
  {
    reset();
    return;
  }

  node_connection_status_->setStatus(prearm_check->node_connection_unstable);
  battery_voltage_status_->setStatus(prearm_check->battery_voltage_too_low);
  cpu_temp_status_->setStatus(prearm_check->cpu_temperature_too_high);
  rotor_comm_status_->setStatus(prearm_check->rotor_communication_error);
  attitude_level_status_->setStatus(prearm_check->attitude_too_steep);
  pos_stability_status_->setStatus(prearm_check->position_unstable);
  pos_accuracy_status_->setStatus(prearm_check->position_inaccurate);
  vel_accuracy_status_->setStatus(prearm_check->velocity_inaccurate);
  atti_accuracy_status_->setStatus(prearm_check->attitude_inaccurate);
  head_accuracy_status_->setStatus(prearm_check->heading_inaccurate);
  ready_status_->setStatus(prearm_check->ok);
}
}  // namespace gcs
}  // namespace gui
