// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/observer.hpp"

#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
ObserverWidget::ObserverWidget()
{
  use_magnetometer_ = new QCheckBox("Use Magnetometer");
  use_magnetometer_->setChecked(true);
  addWidget(use_magnetometer_);

  use_barometer_ = new QCheckBox("Use Barometer");
  use_barometer_->setChecked(false);
  addWidget(use_barometer_);

  use_gnss_ = new QCheckBox("Use GNSS");
  use_gnss_->setChecked(true);
  addWidget(use_gnss_);

  use_ext_pose_ = new QCheckBox("Use External Pose");
  use_ext_pose_->setChecked(false);
  addWidget(use_ext_pose_);

  adaptive_gnss_noise_ = new QCheckBox("Adaptive GNSS Measurement Noise");
  adaptive_gnss_noise_->setChecked(true);
  addWidget(adaptive_gnss_noise_);

  adaptive_grav_noise_ = new QCheckBox("Adaptive Gravity Measurement Noise");
  adaptive_grav_noise_->setChecked(false);
  addWidget(adaptive_grav_noise_);

  do_acc_bias_estimation_ = new QCheckBox("Do Accelerometer Bias Estimation");
  do_acc_bias_estimation_->setChecked(false);
  addWidget(do_acc_bias_estimation_);

  do_gyro_bias_estimation_ = new QCheckBox("Do Gyroscope Bias Estimation");
  do_gyro_bias_estimation_->setChecked(true);
  addWidget(do_gyro_bias_estimation_);

  do_mag_hard_bias_estimation_ = new QCheckBox("Do Magnetometer Hard-Iron Bias Estimation");
  do_mag_hard_bias_estimation_->setChecked(false);
  addWidget(do_mag_hard_bias_estimation_);

  do_mag_soft_bias_estimation_ = new QCheckBox("Do Magnetometer Soft-Iron Bias Estimation");
  do_mag_soft_bias_estimation_->setChecked(false);
  addWidget(do_mag_soft_bias_estimation_);

  do_baro_alt_bias_estimation_ = new QCheckBox("Do Barometer Altitude Bias Estimation");
  do_baro_alt_bias_estimation_->setChecked(true);
  addWidget(do_baro_alt_bias_estimation_);

  do_grav_estimation_ = new QCheckBox("Do Gravity Estimation");
  do_grav_estimation_->setChecked(true);
  addWidget(do_grav_estimation_);

  addStretch();
}

const char* ObserverWidget::name() const
{
  return "State Estimator";
}

const char* ObserverWidget::title() const
{
  return "Set up State Estimator";
}

const char* ObserverWidget::description() const
{
  return "Configure the state estimator. "
         "In most cases, the default values should be left unchanged. "
         "Because modifications can cause the attitude solution to diverge, "
         "make any changes only with appropriate safety precautions and a clear understanding of their impact.";
}

void ObserverWidget::updateInternalDataStructures()
{
  return;
}

bool ObserverWidget::isValid()
{
  if (useGnss() && useExternalPose()) {
    qt::qWarnBox(this, "Multiple sources for 3D position cannot be enabled simultaneously.");
    return false;
  }

  return true;
}

YAML::Node ObserverWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[use_magnetometer_->text()] = use_magnetometer_->isChecked();
  node[use_barometer_->text()] = use_barometer_->isChecked();
  node[use_gnss_->text()] = use_gnss_->isChecked();
  node[use_ext_pose_->text()] = use_ext_pose_->isChecked();
  node[adaptive_gnss_noise_->text()] = adaptive_gnss_noise_->isChecked();
  node[adaptive_grav_noise_->text()] = adaptive_grav_noise_->isChecked();
  node[do_acc_bias_estimation_->text()] = do_acc_bias_estimation_->isChecked();
  node[do_gyro_bias_estimation_->text()] = do_gyro_bias_estimation_->isChecked();
  node[do_mag_hard_bias_estimation_->text()] = do_mag_hard_bias_estimation_->isChecked();
  node[do_mag_soft_bias_estimation_->text()] = do_mag_soft_bias_estimation_->isChecked();
  node[do_baro_alt_bias_estimation_->text()] = do_baro_alt_bias_estimation_->isChecked();
  node[do_grav_estimation_->text()] = do_grav_estimation_->isChecked();

  return node;
}

void ObserverWidget::load(const YAML::Node& node)
{
  use_magnetometer_->setChecked(node[use_magnetometer_->text()].as<bool>());
  use_barometer_->setChecked(node[use_barometer_->text()].as<bool>());
  use_gnss_->setChecked(node[use_gnss_->text()].as<bool>());
  use_ext_pose_->setChecked(node[use_ext_pose_->text()].as<bool>());
  adaptive_gnss_noise_->setChecked(node[adaptive_gnss_noise_->text()].as<bool>());
  adaptive_grav_noise_->setChecked(node[adaptive_grav_noise_->text()].as<bool>());
  do_acc_bias_estimation_->setChecked(node[do_acc_bias_estimation_->text()].as<bool>());
  do_gyro_bias_estimation_->setChecked(node[do_gyro_bias_estimation_->text()].as<bool>());
  do_mag_hard_bias_estimation_->setChecked(node[do_mag_hard_bias_estimation_->text()].as<bool>());
  do_mag_soft_bias_estimation_->setChecked(node[do_mag_soft_bias_estimation_->text()].as<bool>());
  do_baro_alt_bias_estimation_->setChecked(node[do_baro_alt_bias_estimation_->text()].as<bool>());
  do_grav_estimation_->setChecked(node[do_grav_estimation_->text()].as<bool>());
}

bool ObserverWidget::useMagnetometer() const
{
  return use_magnetometer_->isChecked();
}

bool ObserverWidget::useBarometer() const
{
  return use_barometer_->isChecked();
}

bool ObserverWidget::useGnss() const
{
  return use_gnss_->isChecked();
}

bool ObserverWidget::useExternalPose() const
{
  return use_ext_pose_->isChecked();
}

bool ObserverWidget::adaptiveGnssNoise() const
{
  return adaptive_gnss_noise_->isChecked();
}

bool ObserverWidget::adaptiveGravityNoise() const
{
  return adaptive_grav_noise_->isChecked();
}

bool ObserverWidget::doAccelBiasEstimation() const
{
  return do_acc_bias_estimation_->isChecked();
}

bool ObserverWidget::doGyroBiasEstimation() const
{
  return do_gyro_bias_estimation_->isChecked();
}

bool ObserverWidget::doMagHardBiasEstimation() const
{
  return do_mag_hard_bias_estimation_->isChecked();
}

bool ObserverWidget::doMagSoftBiasEstimation() const
{
  return do_mag_soft_bias_estimation_->isChecked();
}

bool ObserverWidget::doBaroAltBiasEstimation() const
{
  return do_baro_alt_bias_estimation_->isChecked();
}

bool ObserverWidget::doGravityEstimation() const
{
  return do_grav_estimation_->isChecked();
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
