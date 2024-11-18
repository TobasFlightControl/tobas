#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>

#include "tobas_setup_assistant/setting_tabs/imu.hpp"

namespace gui
{
namespace setup_assistant
{
IMUWidget::IMUWidget()
{
  offset_ = new ParamGetterWidget_Vector3d("Offset", kSensorOffsetDescription);
  offset_->setSuffix(" m");
  addWidget(offset_);

  update_rate_ = new ParamGetterWidget_SpinBox("Update Rate", "");  // TODO
  update_rate_->setMinimum(1);
  update_rate_->setValue(400);
  update_rate_->setSuffix(" Hz");
  addWidget(update_rate_);

  gyro_noise_density_ = new ParamGetterWidget_SpinBox("Gyroscope Noise Density", "");  // TODO
  gyro_noise_density_->setMinimum(0);
  gyro_noise_density_->setValue(20);
  gyro_noise_density_->setSuffix(" mdps/√Hz");
  addWidget(gyro_noise_density_);

  gyro_random_walk_ = new ParamGetterWidget_DoubleSpinBox("Gyroscope Bias Random Walk", "");  // TODO
  gyro_random_walk_->setDecimals(9);
  gyro_random_walk_->setMinimum(0.);
  gyro_random_walk_->setValue(1e-4);
  gyro_random_walk_->setSuffix(" rad/s^2/√Hz");
  addWidget(gyro_random_walk_);

  gyro_bias_corr_time_ = new ParamGetterWidget_SpinBox("Gyroscope Bias Correlation Time Constant", "");  // TODO
  gyro_bias_corr_time_->setMinimum(0);
  gyro_bias_corr_time_->setValue(1000);
  gyro_bias_corr_time_->setSuffix(" s");
  addWidget(gyro_bias_corr_time_);

  gyro_turn_on_bias_sigma_ =
    new ParamGetterWidget_DoubleSpinBox("Gyroscope Turn On Bias Standard Deviation", "");  // TODO
  gyro_turn_on_bias_sigma_->setDecimals(9);
  gyro_turn_on_bias_sigma_->setMinimum(0.);
  gyro_turn_on_bias_sigma_->setValue(0.05);
  gyro_turn_on_bias_sigma_->setSuffix(" rad/s");
  addWidget(gyro_turn_on_bias_sigma_);

  acc_noise_density_ = new ParamGetterWidget_SpinBox("Accelerometer Noise Density", "");  // TODO
  acc_noise_density_->setMinimum(0);
  acc_noise_density_->setValue(200);
  acc_noise_density_->setSuffix(" ug/√Hz");
  addWidget(acc_noise_density_);

  acc_random_walk_ = new ParamGetterWidget_DoubleSpinBox("Accelerometer Bias Random Walk", "");  // TODO
  acc_random_walk_->setDecimals(9);
  acc_random_walk_->setMinimum(0.);
  acc_random_walk_->setValue(0.01);
  acc_random_walk_->setSuffix(" m/s^3/√Hz");
  addWidget(acc_random_walk_);

  acc_bias_corr_time_ = new ParamGetterWidget_SpinBox("Accelerometer Bias Correlation Time Constant", "");  // TODO
  acc_bias_corr_time_->setMinimum(0);
  acc_bias_corr_time_->setValue(300);
  acc_bias_corr_time_->setSuffix(" s");
  addWidget(acc_bias_corr_time_);

  acc_turn_on_bias_sigma_ =
    new ParamGetterWidget_DoubleSpinBox("Accelerometer Turn On Bias Standard Deviation", "");  // TODO
  acc_turn_on_bias_sigma_->setDecimals(9);
  acc_turn_on_bias_sigma_->setMinimum(0.);
  acc_turn_on_bias_sigma_->setValue(0.2);
  acc_turn_on_bias_sigma_->setSuffix(" m/s^2");
  addWidget(acc_turn_on_bias_sigma_);

  addStretch();
}

const char* IMUWidget::name() const
{
  return "IMU";
}

const char* IMUWidget::title() const
{
  return "Define Inertial Measurement Unit";
}

const char* IMUWidget::description() const
{
  return "";  // TODO
}

void IMUWidget::onOpened()
{
  return;
}

void IMUWidget::updateInternalDataStructures()
{
  return;
}

bool IMUWidget::isValid()
{
  return true;
}

YAML::Node IMUWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[offset_->name()] = offset_->getValue();
  node[update_rate_->name()] = update_rate_->getValue();
  node[gyro_noise_density_->name()] = gyro_noise_density_->getValue();
  node[gyro_random_walk_->name()] = gyro_random_walk_->getValue();
  node[gyro_bias_corr_time_->name()] = gyro_bias_corr_time_->getValue();
  node[gyro_turn_on_bias_sigma_->name()] = gyro_turn_on_bias_sigma_->getValue();
  node[acc_noise_density_->name()] = acc_noise_density_->getValue();
  node[acc_random_walk_->name()] = acc_random_walk_->getValue();
  node[acc_bias_corr_time_->name()] = acc_bias_corr_time_->getValue();
  node[acc_turn_on_bias_sigma_->name()] = acc_turn_on_bias_sigma_->getValue();

  return node;
}

void IMUWidget::load(const YAML::Node& node)
{
  offset_->setValue(node[offset_->name()].as<Eigen::Vector3d>());
  update_rate_->setValue(node[update_rate_->name()].as<int>());
  gyro_noise_density_->setValue(node[gyro_noise_density_->name()].as<double>());
  gyro_random_walk_->setValue(node[gyro_random_walk_->name()].as<double>());
  gyro_bias_corr_time_->setValue(node[gyro_bias_corr_time_->name()].as<int>());
  gyro_turn_on_bias_sigma_->setValue(node[gyro_turn_on_bias_sigma_->name()].as<double>());
  acc_noise_density_->setValue(node[acc_noise_density_->name()].as<double>());
  acc_random_walk_->setValue(node[acc_random_walk_->name()].as<double>());
  acc_bias_corr_time_->setValue(node[acc_bias_corr_time_->name()].as<int>());
  acc_turn_on_bias_sigma_->setValue(node[acc_turn_on_bias_sigma_->name()].as<double>());
}

Eigen::Vector3d IMUWidget::offset() const
{
  return offset_->getValue();
}

int IMUWidget::updateRate() const
{
  return update_rate_->getValue();
}

double IMUWidget::gyroNoiseDensity() const
{
  return (tobas_std::kDeg2Rad * 1e-3) * gyro_noise_density_->getValue();
}

double IMUWidget::gyroRandomWalk() const
{
  return gyro_random_walk_->getValue();
}

int IMUWidget::gyroBiasCorrTime() const
{
  return gyro_bias_corr_time_->getValue();
}

double IMUWidget::gyroTurnOnBiasSigma() const
{
  return gyro_turn_on_bias_sigma_->getValue();
}

double IMUWidget::accNoiseDensity() const
{
  return (tobas_std::kGravity * 1e-6) * acc_noise_density_->getValue();
}

double IMUWidget::accRandomWalk() const
{
  return acc_random_walk_->getValue();
}

int IMUWidget::accBiasCorrTime() const
{
  return acc_bias_corr_time_->getValue();
}

double IMUWidget::accTurnOnBiasSigma() const
{
  return acc_turn_on_bias_sigma_->getValue();
}
}  // namespace setup_assistant
}  // namespace gui
