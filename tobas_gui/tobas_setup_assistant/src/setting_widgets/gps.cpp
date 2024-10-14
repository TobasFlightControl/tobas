#include <tobas_yaml_tools/convert/eigen.hpp>

#include "tobas_setup_assistant/setting_tabs/gps.hpp"

namespace gui
{
namespace setup_assistant
{
GPSWidget::GPSWidget()
{
  offset_ = new ParamGetterWidget_Vector3d("Offset", kSensorOffsetDescription);
  offset_->setSuffix(" m");
  addParamWidget(offset_);

  update_rate_ = new ParamGetterWidget_SpinBox("Update Rate", "");  // TODO
  update_rate_->setMinimum(1);
  update_rate_->setValue(5);
  update_rate_->setSuffix(" Hz");
  addParamWidget(update_rate_);

  delay_ = new ParamGetterWidget_DoubleSpinBox("Communication Delay", "");  // TODO
  delay_->setDecimals(2);
  delay_->setMinimum(0.);
  delay_->setValue(0.2);
  delay_->setSuffix(" s");
  addParamWidget(delay_);

  pos_corr_time_ = new ParamGetterWidget_SpinBox("Position Correction Time Constant", "");  // TODO
  pos_corr_time_->setMinimum(1);
  pos_corr_time_->setValue(10);
  pos_corr_time_->setSuffix(" s");
  addParamWidget(pos_corr_time_);

  horizontal_pos_accuracy_ = new ParamGetterWidget_DoubleSpinBox("Horizontal Position Accuracy", "");  // TODO
  horizontal_pos_accuracy_->setDecimals(2);
  horizontal_pos_accuracy_->setMinimum(0.);
  horizontal_pos_accuracy_->setValue(2.);
  horizontal_pos_accuracy_->setSuffix(" m");
  addParamWidget(horizontal_pos_accuracy_);

  vertical_pos_accuracy_ = new ParamGetterWidget_DoubleSpinBox("Vertical Position Accuracy", "");  // TODO
  vertical_pos_accuracy_->setDecimals(2);
  vertical_pos_accuracy_->setMinimum(0.);
  vertical_pos_accuracy_->setValue(4.);
  vertical_pos_accuracy_->setSuffix(" m");
  addParamWidget(vertical_pos_accuracy_);

  horizontal_vel_stddev_ =
    new ParamGetterWidget_DoubleSpinBox("Standard Deviation for Horizontal Speed Noise", "");  // TODO
  horizontal_vel_stddev_->setDecimals(2);
  horizontal_vel_stddev_->setMinimum(0.);
  horizontal_vel_stddev_->setValue(0.1);
  horizontal_vel_stddev_->setSuffix(" m/s");
  addParamWidget(horizontal_vel_stddev_);

  vertical_vel_stddev_ =
    new ParamGetterWidget_DoubleSpinBox("Standard Deviation for Vertical Speed Noise", "");  // TODO
  vertical_vel_stddev_->setDecimals(2);
  vertical_vel_stddev_->setMinimum(0.);
  vertical_vel_stddev_->setValue(0.1);
  vertical_vel_stddev_->setSuffix(" m/s");
  addParamWidget(vertical_vel_stddev_);

  addStretch();
}

const char* GPSWidget::name() const
{
  return "GPS";
}

const char* GPSWidget::title() const
{
  return "Define Global Positioning System";
}

const char* GPSWidget::description() const
{
  return "";  // TODO
}

void GPSWidget::onOpened()
{
  return;
}

void GPSWidget::updateInternalDataStructures()
{
  return;
}

bool GPSWidget::isValid()
{
  if (!equipped())
    return true;

  return true;
}

YAML::Node GPSWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[kEquippedKey] = equipped_->isChecked();

  node[offset_->name()] = offset_->getValue();
  node[update_rate_->name()] = update_rate_->getValue();
  node[delay_->name()] = delay_->getValue();
  node[pos_corr_time_->name()] = pos_corr_time_->getValue();
  node[horizontal_pos_accuracy_->name()] = horizontal_pos_accuracy_->getValue();
  node[vertical_pos_accuracy_->name()] = vertical_pos_accuracy_->getValue();
  node[horizontal_vel_stddev_->name()] = horizontal_vel_stddev_->getValue();
  node[vertical_vel_stddev_->name()] = vertical_vel_stddev_->getValue();

  return node;
}

void GPSWidget::load(const YAML::Node& node)
{
  equipped_->setChecked(node[kEquippedKey].as<bool>());

  offset_->setValue(node[offset_->name()].as<Eigen::Vector3d>());
  update_rate_->setValue(node[update_rate_->name()].as<int>());
  delay_->setValue(node[delay_->name()].as<double>());
  pos_corr_time_->setValue(node[pos_corr_time_->name()].as<int>());
  horizontal_pos_accuracy_->setValue(node[horizontal_pos_accuracy_->name()].as<double>());
  vertical_pos_accuracy_->setValue(node[vertical_pos_accuracy_->name()].as<double>());
  horizontal_vel_stddev_->setValue(node[horizontal_vel_stddev_->name()].as<double>());
  vertical_vel_stddev_->setValue(node[vertical_vel_stddev_->name()].as<double>());
}

Eigen::Vector3d GPSWidget::offset() const
{
  return offset_->getValue();
}

int GPSWidget::updateRate() const
{
  return update_rate_->getValue();
}

double GPSWidget::delay() const
{
  return delay_->getValue();
}

int GPSWidget::positionCorrectionTime() const
{
  return pos_corr_time_->getValue();
}

double GPSWidget::horizontalPositionAccuracy() const
{
  return horizontal_pos_accuracy_->getValue();
}

double GPSWidget::verticalPositionAccuracy() const
{
  return vertical_pos_accuracy_->getValue();
}

double GPSWidget::horizontalVelocityStddev() const
{
  return horizontal_vel_stddev_->getValue();
}

double GPSWidget::verticalVelocityStddev() const
{
  return vertical_vel_stddev_->getValue();
}

bool GPSWidget::defaultEquipped() const
{
  return true;
}
}  // namespace setup_assistant
}  // namespace gui
