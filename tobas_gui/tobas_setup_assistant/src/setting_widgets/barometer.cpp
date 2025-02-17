#include <tobas_yaml_tools/convert/eigen.hpp>

#include "tobas_setup_assistant/setting_tabs/barometer.hpp"

namespace gui
{
namespace sa
{
BarometerWidget::BarometerWidget()
{
  offset_ = new ParamGetterWidget_Vector3d("Offset", kSensorOffsetDescription);
  offset_->setSuffix(" m");
  addWidget(offset_);

  update_rate_ = new ParamGetterWidget_SpinBox("Update Rate", "");  // TODO
  update_rate_->setMinimum(1);
  update_rate_->setValue(50);
  update_rate_->setSuffix(" Hz");
  addWidget(update_rate_);

  pressure_var_ = new ParamGetterWidget_DoubleSpinBox("Air Pressure Variance", "");  // TODO
  pressure_var_->setDecimals(2);
  pressure_var_->setMinimum(0.);
  pressure_var_->setValue(10.);
  pressure_var_->setSuffix(" Pa^2");
  addWidget(pressure_var_);

  addStretch();
}

const char* BarometerWidget::name() const
{
  return "Barometer";
}

const char* BarometerWidget::title() const
{
  return "Define Air Pressure Sensor";
}

const char* BarometerWidget::description() const
{
  return "";  // TODO
}

void BarometerWidget::onOpened()
{
  return;
}

void BarometerWidget::updateInternalDataStructures()
{
  return;
}

bool BarometerWidget::isValid()
{
  return true;
}

YAML::Node BarometerWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[offset_->name()] = offset_->getValue();
  node[update_rate_->name()] = update_rate_->getValue();
  node[pressure_var_->name()] = pressure_var_->getValue();

  return node;
}

void BarometerWidget::load(const YAML::Node& node)
{
  offset_->setValue(node[offset_->name()].as<Eigen::Vector3d>());
  update_rate_->setValue(node[update_rate_->name()].as<int>());
  pressure_var_->setValue(node[pressure_var_->name()].as<double>());
}

Eigen::Vector3d BarometerWidget::offset() const
{
  return offset_->getValue();
}

int BarometerWidget::updateRate() const
{
  return update_rate_->getValue();
}

double BarometerWidget::pressureVariance() const
{
  return pressure_var_->getValue();
}
}  // namespace sa
}  // namespace gui
