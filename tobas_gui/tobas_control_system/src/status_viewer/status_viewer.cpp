#include <QVBoxLayout>

#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>

#include "tobas_control_system/status_viewer/status_viewer.hpp"

namespace gui
{
namespace control_system
{
StatusViewerWidget::StatusViewerWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  gps_status_ = new StatusWidget("GPS 3D Fix");
  rcin_status_ = new StatusWidget("Radio Input");
  voltage_status_ = new StatusWidget("Battery Voltage");
  attitude_status_ = new StatusWidget("Attitude Horizontal");
  pos_stability_status_ = new StatusWidget("Position Stable");
  pos_accuracy_status_ = new StatusWidget("Position Estimation Accurate");
  rot_accuracy_status_ = new StatusWidget("Orientation Estimation Accurate");
  vel_accuracy_status_ = new StatusWidget("Velocity Estimation Accurate");
  ready_status_ = new StatusWidget("Ready to Arm");
  arming_status_ = new StatusWidget("Rotors Armed");

  const auto rows = new QVBoxLayout();
  rows->addWidget(gps_status_);
  rows->addWidget(rcin_status_);
  rows->addWidget(voltage_status_);
  rows->addWidget(attitude_status_);
  rows->addWidget(pos_stability_status_);
  rows->addWidget(pos_accuracy_status_);
  rows->addWidget(rot_accuracy_status_);
  rows->addWidget(vel_accuracy_status_);
  rows->addWidget(ready_status_);
  rows->addWidget(arming_status_);
  rows->addStretch();

  setLayout(rows);
}

void StatusViewerWidget::updateNamespace(const std::string& ns)
{
  gps_status_->reset();
  rcin_status_->reset();
  voltage_status_->reset();
  attitude_status_->reset();
  pos_stability_status_->reset();
  pos_accuracy_status_->reset();
  rot_accuracy_status_->reset();
  vel_accuracy_status_->reset();
  ready_status_->reset();
  arming_status_->reset();

  gps_sub_ = ros2::createSubscriber(node_, path::join(ns, tobas::kGpsTopic), &self::gpsCb, this);
  rcin_sub_ = ros2::createSubscriber(node_, path::join(ns, tobas::kRcInputTopic), &self::rcInputCb, this);
  prearm_check_sub_ =
    ros2::createSubscriber(node_, path::join(ns, tobas::kPreArmCheckTopic), &self::preArmCheckCb, this);
  arming_sub_ = ros2::createSubscriber(node_, path::join(ns, tobas::kArmingTopic), &self::armingCb, this);
}

void StatusViewerWidget::gpsCb(const tobas_msgs::Gps::ConstSharedPtr& gps)
{
  gps_status_->setStatus(gps->fix_type == tobas_msgs::msg::Gps::FIX_3D);
}

void StatusViewerWidget::rcInputCb(const tobas_msgs::msg::RCInput::ConstSharedPtr&)
{
  rcin_status_->setStatus(true);
}

void StatusViewerWidget::preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check)
{
  voltage_status_->setStatus(prearm_check->battery_voltage_sufficient);
  attitude_status_->setStatus(prearm_check->attitude_horizontal);
  pos_stability_status_->setStatus(prearm_check->position_stable);
  pos_accuracy_status_->setStatus(prearm_check->position_accurate);
  rot_accuracy_status_->setStatus(prearm_check->orientation_accurate);
  vel_accuracy_status_->setStatus(prearm_check->velocity_accurate);
  ready_status_->setStatus(prearm_check->ok);
}

void StatusViewerWidget::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_status_->setStatus(arming->data);
}
}  // namespace control_system
}  // namespace gui
