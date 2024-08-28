#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>

#include "tobas_setup_assistant/setting_widgets/barometer.hpp"

namespace gui
{
namespace setup_assistant
{
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

void BarometerWidget::onInit()
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
  offset_->setValue(yaml::load<Eigen::Vector3d>(offset_->name(), node));
  update_rate_->setValue(yaml::load<int>(update_rate_->name(), node));
  pressure_var_->setValue(yaml::load<double>(pressure_var_->name(), node));
}
}  // namespace setup_assistant
}  // namespace gui
