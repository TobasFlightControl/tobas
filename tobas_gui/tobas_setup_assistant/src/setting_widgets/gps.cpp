#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>

#include "tobas_setup_assistant/setting_widgets/gps.hpp"

namespace gui
{
namespace setup_assistant
{
const char* GpsWidget::name()
{
  return "GPS";
}

const char* GpsWidget::title()
{
  return "Define Global Positioning System";
}

const char* GpsWidget::description()
{
  return "";  // TODO
}

void GpsWidget::onInit()
{
  offset_ = new ParamGetterWidget_Vector3d("Offset", kSensorOffsetDescription);
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

  pos_corr_time_ = new ParamGetterWidget_SpinBox("Communication Delay", "");  // TODO
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
}

void GpsWidget::onOpened()
{
  return;
}

void GpsWidget::updateInternalDataStructures()
{
  return;
}

bool GpsWidget::isValid()
{
  if (!equipped())
    return true;

  return true;
}

YAML::Node GpsWidget::dump()
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

void GpsWidget::load(const YAML::Node& node)
{
  equipped_->setChecked(yaml::load<bool>(kEquippedKey, node));

  offset_->setValue(yaml::load<Eigen::Vector3d>(offset_->name(), node));
  update_rate_->setValue(yaml::load<int>(update_rate_->name(), node));
  delay_->setValue(yaml::load<double>(delay_->name(), node));
  pos_corr_time_->setValue(yaml::load<int>(pos_corr_time_->name(), node));
  horizontal_pos_accuracy_->setValue(yaml::load<double>(horizontal_pos_accuracy_->name(), node));
  vertical_pos_accuracy_->setValue(yaml::load<double>(vertical_pos_accuracy_->name(), node));
  horizontal_vel_stddev_->setValue(yaml::load<double>(horizontal_vel_stddev_->name(), node));
  vertical_vel_stddev_->setValue(yaml::load<double>(vertical_vel_stddev_->name(), node));
}

bool GpsWidget::defaultEquipped() const
{
  return true;
}
}  // namespace setup_assistant
}  // namespace gui
