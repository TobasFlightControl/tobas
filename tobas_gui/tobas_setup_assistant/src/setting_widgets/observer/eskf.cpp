#include <tobas_yaml_tools/convert/eigen.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/observer/eskf.hpp"

namespace gui
{
namespace setup_assistant
{
ErrorStateKalmanFilterWidget::ErrorStateKalmanFilterWidget(
  const RobotInfo& robot,
  const IMUWidget* imu,
  const BarometerWidget* baro,
  const GPSWidget* gps)
  : robot_(robot), imu_(imu), baro_(baro), gps_(gps)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  do_acc_bias_estimation_ = new QCheckBox("Do Accelerometer Bias Estimation");
  do_acc_bias_estimation_->setChecked(false);
  rows->addWidget(do_acc_bias_estimation_);

  do_gyro_bias_estimation_ = new QCheckBox("Do Gyroscope Bias Estimation");
  do_gyro_bias_estimation_->setChecked(true);
  rows->addWidget(do_gyro_bias_estimation_);

  do_mag_hard_bias_estimation_ = new QCheckBox("Do Magnetometer Hard-Iron Bias Estimation");
  do_mag_hard_bias_estimation_->setChecked(true);
  rows->addWidget(do_mag_hard_bias_estimation_);

  do_mag_soft_bias_estimation_ = new QCheckBox("Do Magnetometer Soft-Iron Bias Estimation");
  do_mag_soft_bias_estimation_->setChecked(true);
  rows->addWidget(do_mag_soft_bias_estimation_);

  do_grav_estimation_ = new QCheckBox("Do Gravity Estimation");
  do_grav_estimation_->setChecked(true);
  rows->addWidget(do_grav_estimation_);

  rows->addStretch();
}

const char* ErrorStateKalmanFilterWidget::name() const
{
  return "Error State Kalman Filter";
}

const char* ErrorStateKalmanFilterWidget::description() const
{
  return "The Error State Kalman Filter (ESKF) is an advanced variant of the Kalman Filter, "
         "tailored for systems with non-linear dynamics. "
         "Unlike the traditional Kalman Filter, which directly estimates the system's state, "
         "the ESKF focuses on estimating the error in the state. "
         "This approach allows for more effective handling of non-linear relationships "
         "between the system state and measurements. "
         "The ESKF operates by linearizing these non-linearities around a nominal state. "
         "It's particularly useful in applications like navigation and tracking, "
         "where precision in estimating orientation and position is crucial, "
         "such as in Inertial Navigation Systems and GPS technology. "
         "The ESKF's blend of accuracy and computational efficiency "
         "makes it a valuable tool in complex engineering tasks.";
}

QString ErrorStateKalmanFilterWidget::observerPackage() const
{
  return "tobas_eskf";
}

QString ErrorStateKalmanFilterWidget::pluginName() const
{
  return "ObserverNode";
}

YAML::Node ErrorStateKalmanFilterWidget::staticParams() const
{
  YAML::Node node(YAML::NodeType::Map);

  node["frame_id"] = robot_.tree().getRootName();
  node["use_barometer"] = false;  // TODO: 選択できるように
  node["use_gps"] = gps_->equipped();
  node["do_acc_bias_estimation"] = do_acc_bias_estimation_->isChecked();
  node["do_gyro_bias_estimation"] = do_gyro_bias_estimation_->isChecked();
  node["do_mag_hard_bias_estimation"] = do_mag_hard_bias_estimation_->isChecked();
  node["do_mag_soft_bias_estimation"] = do_mag_soft_bias_estimation_->isChecked();
  node["do_gravity_estimation"] = do_grav_estimation_->isChecked();
  node["imu_offset"] = imu_->offset();
  node["barometer_offset"] = baro_->offset();
  node["gps_offset"] = gps_->offset();

  return node;
}

YAML::Node ErrorStateKalmanFilterWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[do_acc_bias_estimation_->text()] = do_acc_bias_estimation_->isChecked();
  node[do_gyro_bias_estimation_->text()] = do_gyro_bias_estimation_->isChecked();
  node[do_mag_hard_bias_estimation_->text()] = do_mag_hard_bias_estimation_->isChecked();
  node[do_mag_soft_bias_estimation_->text()] = do_mag_soft_bias_estimation_->isChecked();
  node[do_grav_estimation_->text()] = do_grav_estimation_->isChecked();

  return node;
}

void ErrorStateKalmanFilterWidget::load(const YAML::Node& node)
{
  do_acc_bias_estimation_->setChecked(node[do_acc_bias_estimation_->text()].as<bool>());
  do_gyro_bias_estimation_->setChecked(node[do_gyro_bias_estimation_->text()].as<bool>());
  do_mag_hard_bias_estimation_->setChecked(node[do_mag_hard_bias_estimation_->text()].as<bool>());
  do_mag_soft_bias_estimation_->setChecked(node[do_mag_soft_bias_estimation_->text()].as<bool>());
  do_grav_estimation_->setChecked(node[do_grav_estimation_->text()].as<bool>());
}

bool ErrorStateKalmanFilterWidget::isValid()
{
  // 絶対位置が取得できることを確認
  if (!gps_->equipped())
  {
    qt::qErrorBox(this, "Absolute position connot be observed. Please review the sensor settings.");
    return false;
  }

  return true;
}
}  // namespace setup_assistant
}  // namespace gui
