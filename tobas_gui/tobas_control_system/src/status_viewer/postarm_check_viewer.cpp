#include "tobas_control_system/status_viewer/postarm_check_viewer.hpp"

#include <QVBoxLayout>

namespace gui
{
namespace gcs
{
PostArmCheckViewerWidget::PostArmCheckViewerWidget(const RosQtBridge& bridge)
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
  connect(&bridge, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::postArmCheckReceived, this, &self::postArmCheckCb, Qt::QueuedConnection);
}

void PostArmCheckViewerWidget::reset()
{
  gyro_noise_status_->reset();
  accel_noise_status_->reset();
  mag_offset_status_->reset();
  mag_alignment_status_->reset();
  latency_status_->reset();
}

void PostArmCheckViewerWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;

  if (arming->data) {
    setEnabled(true);
  }
  else {
    reset();
    setEnabled(false);
  }
}

void PostArmCheckViewerWidget::postArmCheckCb(const tobas_msgs::msg::PostArmCheck::ConstSharedPtr& postarm_check)
{
  if (!arming_) {
    return;
  }

  if (!arming_->data) {
    return;
  }

  gyro_noise_status_->setStatus(!postarm_check->gyro_noise_too_large);
  accel_noise_status_->setStatus(!postarm_check->accel_noise_too_large);
  mag_offset_status_->setStatus(!postarm_check->mag_offset_too_large);
  mag_alignment_status_->setStatus(!postarm_check->mag_misalignment);
  latency_status_->setStatus(!postarm_check->latency_too_large);
}
}  // namespace gcs
}  // namespace gui
