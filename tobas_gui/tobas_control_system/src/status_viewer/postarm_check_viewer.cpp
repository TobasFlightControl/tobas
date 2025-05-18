#include "tobas_control_system/status_viewer/postarm_check_viewer.hpp"

#include <QVBoxLayout>

#include <tobas_constants/constants.hpp>
#include <tobas_path_tools/join.hpp>

namespace gui
{
namespace gcs
{
PostArmCheckViewerWidget::PostArmCheckViewerWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  gyro_noise_status_ = new StatusWidget("Gyroscope Noise Level");
  accel_noise_status_ = new StatusWidget("Accelerometer Noise Level");
  mag_offset_status_ = new StatusWidget("Magnetic Field Offset");
  mag_alignment_status_ = new StatusWidget("Magnetic Field Alignment");
  latency_status_ = new StatusWidget("Control Latency");

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(gyro_noise_status_);
  rows->addWidget(accel_noise_status_);
  rows->addWidget(mag_offset_status_);
  rows->addWidget(mag_alignment_status_);
  rows->addWidget(latency_status_);
  rows->addStretch();
  setLayout(rows);

  // Connection
  connect(this, &self::armingReceived, this, &self::armingCbQt, Qt::QueuedConnection);
  connect(this, &self::postArmCheckReceived, this, &self::postArmCheckCbQt, Qt::QueuedConnection);
}

void PostArmCheckViewerWidget::reset()
{
  gyro_noise_status_->reset();
  accel_noise_status_->reset();
  mag_offset_status_->reset();
  mag_alignment_status_->reset();
  latency_status_->reset();
}

void PostArmCheckViewerWidget::updateNamespace(const std::string& ns)
{
  reset();

  arming_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kArmingTopic), &self::armingCbRos, this);
  postarm_check_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kPostArmCheckTopic), &self::postArmCheckCbRos, this);
}

void PostArmCheckViewerWidget::armingCbRos(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
  Q_EMIT armingReceived(arming->data);
}

void PostArmCheckViewerWidget::postArmCheckCbRos(const tobas_msgs::msg::PostArmCheck::ConstSharedPtr& postarm_check)
{
  if (!arming_) {
    return;
  }

  if (!arming_->data) {
    return;
  }

  Q_EMIT postArmCheckReceived(
    postarm_check->gyro_noise_too_large,
    postarm_check->accel_noise_too_large,
    postarm_check->mag_offset_too_large,
    postarm_check->mag_misalignment,
    postarm_check->latency_too_large);
}

void PostArmCheckViewerWidget::armingCbQt(bool arming)
{
  if (arming) {
    setEnabled(true);
  }
  else {
    reset();
    setEnabled(false);
  }
}

void PostArmCheckViewerWidget::postArmCheckCbQt(
  bool gyro_noise_too_large,
  bool accel_noise_too_large,
  bool mag_offset_too_large,
  bool mag_misalignment,
  bool latency_too_large)
{
  gyro_noise_status_->setStatus(!gyro_noise_too_large);
  accel_noise_status_->setStatus(!accel_noise_too_large);
  mag_offset_status_->setStatus(!mag_offset_too_large);
  mag_alignment_status_->setStatus(!mag_misalignment);
  latency_status_->setStatus(!latency_too_large);
}
}  // namespace gcs
}  // namespace gui
