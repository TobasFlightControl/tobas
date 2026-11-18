// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/observer.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace tobas::gui::sa
{
namespace
{
constexpr char kAdaptiveGnssNoiseLabel[] = "Enable Adaptive GNSS Measurement Noise";
constexpr char kAdaptiveGravityNoiseLabel[] = "Enable Adaptive Gravity Measurement Noise";
constexpr char kAccelBiasEstimationLabel[] = "Estimate Accelerometer Bias";
constexpr char kGyroBiasEstimationLabel[] = "Estimate Gyroscope Bias";
constexpr char kMagHardBiasEstimationLabel[] = "Estimate Magnetometer Hard-Iron Bias";
constexpr char kMagSoftBiasEstimationLabel[] = "Estimate Magnetometer Soft-Iron Bias";
constexpr char kBaroAltitudeBiasEstimationLabel[] = "Estimate Barometric Altitude Bias";
constexpr char kGravityEstimationLabel[] = "Estimate Gravity";
}  // namespace

ObserverWidget::ObserverWidget()
{
  const auto rows = new QVBoxLayout(this);

  use_magnetometer_ = new QCheckBox("Use Magnetometer");
  rows->addWidget(use_magnetometer_);

  use_barometer_ = new QCheckBox("Use Barometer");
  rows->addWidget(use_barometer_);

  use_gnss_ = new QCheckBox("Use GNSS");
  rows->addWidget(use_gnss_);

  use_ext_pose_ = new QCheckBox("Use External Pose");
  rows->addWidget(use_ext_pose_);

  adaptive_gnss_noise_ = new QCheckBox(kAdaptiveGnssNoiseLabel);
  rows->addWidget(adaptive_gnss_noise_);

  adaptive_grav_noise_ = new QCheckBox(kAdaptiveGravityNoiseLabel);
  rows->addWidget(adaptive_grav_noise_);

  do_acc_bias_estimation_ = new QCheckBox(kAccelBiasEstimationLabel);
  rows->addWidget(do_acc_bias_estimation_);

  do_gyro_bias_estimation_ = new QCheckBox(kGyroBiasEstimationLabel);
  rows->addWidget(do_gyro_bias_estimation_);

  do_mag_hard_bias_estimation_ = new QCheckBox(kMagHardBiasEstimationLabel);
  rows->addWidget(do_mag_hard_bias_estimation_);

  do_mag_soft_bias_estimation_ = new QCheckBox(kMagSoftBiasEstimationLabel);
  rows->addWidget(do_mag_soft_bias_estimation_);

  do_baro_alt_bias_estimation_ = new QCheckBox(kBaroAltitudeBiasEstimationLabel);
  rows->addWidget(do_baro_alt_bias_estimation_);

  do_grav_estimation_ = new QCheckBox(kGravityEstimationLabel);
  rows->addWidget(do_grav_estimation_);

  rows->addStretch(1);
}

const char* ObserverWidget::name() const
{
  return "State Estimator";
}

const char* ObserverWidget::title() const
{
  return "Configure State Estimator";
}

const char* ObserverWidget::description() const
{
  return "Select the measurement sources and online bias estimation used by the state estimator. "
         "GNSS and external pose cannot be enabled at the same time. "
         "Use the defaults unless your sensors or estimation requirements call for different settings.";
}

void ObserverWidget::updateInternalDataStructures()
{
  return;
}

void ObserverWidget::setToDefaults()
{
  use_magnetometer_->setChecked(true);
  use_barometer_->setChecked(true);
  use_gnss_->setChecked(true);
  use_ext_pose_->setChecked(false);
  adaptive_gnss_noise_->setChecked(true);
  adaptive_grav_noise_->setChecked(false);
  do_acc_bias_estimation_->setChecked(false);
  do_gyro_bias_estimation_->setChecked(true);
  do_mag_hard_bias_estimation_->setChecked(false);
  do_mag_soft_bias_estimation_->setChecked(false);
  do_baro_alt_bias_estimation_->setChecked(true);
  do_grav_estimation_->setChecked(true);
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
  node[kAdaptiveGnssNoiseLabel] = adaptive_gnss_noise_->isChecked();
  node[kAdaptiveGravityNoiseLabel] = adaptive_grav_noise_->isChecked();
  node[kAccelBiasEstimationLabel] = do_acc_bias_estimation_->isChecked();
  node[kGyroBiasEstimationLabel] = do_gyro_bias_estimation_->isChecked();
  node[kMagHardBiasEstimationLabel] = do_mag_hard_bias_estimation_->isChecked();
  node[kMagSoftBiasEstimationLabel] = do_mag_soft_bias_estimation_->isChecked();
  node[kBaroAltitudeBiasEstimationLabel] = do_baro_alt_bias_estimation_->isChecked();
  node[kGravityEstimationLabel] = do_grav_estimation_->isChecked();

  return node;
}

void ObserverWidget::load(const YAML::Node& _node)
{
  use_magnetometer_->setChecked(_node[use_magnetometer_->text()].as<bool>());
  use_barometer_->setChecked(_node[use_barometer_->text()].as<bool>());
  use_gnss_->setChecked(_node[use_gnss_->text()].as<bool>());
  use_ext_pose_->setChecked(_node[use_ext_pose_->text()].as<bool>());
  adaptive_gnss_noise_->setChecked(_node[kAdaptiveGnssNoiseLabel].as<bool>());
  adaptive_grav_noise_->setChecked(_node[kAdaptiveGravityNoiseLabel].as<bool>());
  do_acc_bias_estimation_->setChecked(_node[kAccelBiasEstimationLabel].as<bool>());
  do_gyro_bias_estimation_->setChecked(_node[kGyroBiasEstimationLabel].as<bool>());
  do_mag_hard_bias_estimation_->setChecked(_node[kMagHardBiasEstimationLabel].as<bool>());
  do_mag_soft_bias_estimation_->setChecked(_node[kMagSoftBiasEstimationLabel].as<bool>());
  do_baro_alt_bias_estimation_->setChecked(_node[kBaroAltitudeBiasEstimationLabel].as<bool>());
  do_grav_estimation_->setChecked(_node[kGravityEstimationLabel].as<bool>());
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
}  // namespace tobas::gui::sa
