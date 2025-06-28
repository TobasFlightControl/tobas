#include "tobas_setup_assistant/setting_tabs/observer.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace gui
{
namespace sa
{
ObserverWidget::ObserverWidget()
{
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

  do_grav_estimation_ = new QCheckBox("Do Gravity Estimation");
  do_grav_estimation_->setChecked(true);
  addWidget(do_grav_estimation_);

  addStretch();
}

const char* ObserverWidget::name() const
{
  return "Observer";
}

const char* ObserverWidget::title() const
{
  return "Setup Observer";
}

const char* ObserverWidget::description() const
{
  return "";  // TODO
}

void ObserverWidget::onOpened()
{
  return;
}

void ObserverWidget::updateInternalDataStructures()
{
  return;
}

bool ObserverWidget::isValid()
{
  return true;
}

YAML::Node ObserverWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[adaptive_gnss_noise_->text()] = adaptive_gnss_noise_->isChecked();
  node[adaptive_grav_noise_->text()] = adaptive_grav_noise_->isChecked();
  node[do_acc_bias_estimation_->text()] = do_acc_bias_estimation_->isChecked();
  node[do_gyro_bias_estimation_->text()] = do_gyro_bias_estimation_->isChecked();
  node[do_mag_hard_bias_estimation_->text()] = do_mag_hard_bias_estimation_->isChecked();
  node[do_mag_soft_bias_estimation_->text()] = do_mag_soft_bias_estimation_->isChecked();
  node[do_grav_estimation_->text()] = do_grav_estimation_->isChecked();

  return node;
}

void ObserverWidget::load(const YAML::Node& node)
{
  adaptive_gnss_noise_->setChecked(node[adaptive_gnss_noise_->text()].as<bool>());
  adaptive_grav_noise_->setChecked(node[adaptive_grav_noise_->text()].as<bool>());
  do_acc_bias_estimation_->setChecked(node[do_acc_bias_estimation_->text()].as<bool>());
  do_gyro_bias_estimation_->setChecked(node[do_gyro_bias_estimation_->text()].as<bool>());
  do_mag_hard_bias_estimation_->setChecked(node[do_mag_hard_bias_estimation_->text()].as<bool>());
  do_mag_soft_bias_estimation_->setChecked(node[do_mag_soft_bias_estimation_->text()].as<bool>());
  do_grav_estimation_->setChecked(node[do_grav_estimation_->text()].as<bool>());
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

bool ObserverWidget::doGravityEstimation() const
{
  return do_grav_estimation_->isChecked();
}
}  // namespace sa
}  // namespace gui
