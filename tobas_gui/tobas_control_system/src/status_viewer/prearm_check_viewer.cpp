#include <QVBoxLayout>

#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>

#include "tobas_control_system/status_viewer/prearm_check_viewer.hpp"

namespace gui
{
namespace control_system
{
PreArmCheckViewerWidget::PreArmCheckViewerWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  voltage_status_ = new StatusWidget("Battery Voltage");
  cpu_status_ = new StatusWidget("CPU Temperature");
  rotors_status_ = new StatusWidget("Rotors Communication");
  attitude_status_ = new StatusWidget("Attitude Horizontal");
  pos_stability_status_ = new StatusWidget("Position Stable");
  pos_accuracy_status_ = new StatusWidget("Position Estimation Accurate");
  rot_accuracy_status_ = new StatusWidget("Orientation Estimation Accurate");
  vel_accuracy_status_ = new StatusWidget("Velocity Estimation Accurate");
  ready_status_ = new StatusWidget("Ready to Arm");

  const auto rows = new QVBoxLayout();
  rows->addWidget(voltage_status_);
  rows->addWidget(cpu_status_);
  rows->addWidget(rotors_status_);
  rows->addWidget(attitude_status_);
  rows->addWidget(pos_stability_status_);
  rows->addWidget(pos_accuracy_status_);
  rows->addWidget(rot_accuracy_status_);
  rows->addWidget(vel_accuracy_status_);
  rows->addWidget(ready_status_);
  rows->addStretch();

  setLayout(rows);
}

void PreArmCheckViewerWidget::reset()
{
  voltage_status_->reset();
  cpu_status_->reset();
  rotors_status_->reset();
  attitude_status_->reset();
  pos_stability_status_->reset();
  pos_accuracy_status_->reset();
  rot_accuracy_status_->reset();
  vel_accuracy_status_->reset();
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
  if (arming_ == nullptr)
    return;
  if (arming_->data)
    return;

  voltage_status_->setStatus(!prearm_check->battery_voltage_too_low);
  cpu_status_->setStatus(!prearm_check->cpu_temperature_too_high);
  rotors_status_->setStatus(!prearm_check->rotor_communication_error);
  attitude_status_->setStatus(!prearm_check->attitude_too_steep);
  pos_stability_status_->setStatus(!prearm_check->position_unstable);
  pos_accuracy_status_->setStatus(!prearm_check->position_inaccurate);
  rot_accuracy_status_->setStatus(!prearm_check->orientation_inaccurate);
  vel_accuracy_status_->setStatus(!prearm_check->velocity_inaccurate);
  ready_status_->setStatus(prearm_check->ok);
}
}  // namespace control_system
}  // namespace gui
