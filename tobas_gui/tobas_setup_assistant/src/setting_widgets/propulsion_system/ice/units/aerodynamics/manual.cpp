#include <tobas_qt_tools/cast.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/aerodynamics/manual.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
AerodynamicsWidget_Manual::AerodynamicsWidget_Manual()
{
  motor_const_ = new ParamGetterWidget_LinearEquation("Motor Constant", "", "CT", "φ");  // TODO
  motor_const_->setDecimals(9);
  motor_const_->setValue({ 4.471e-3, 1.887e-2 });
  motor_const_->setSuffix(" kg m/rad^2");
  addWidget(motor_const_);

  moment_const_ = new ParamGetterWidget_DoubleSpinBox("Moment Constant", "");  // TODO
  moment_const_->setDecimals(6);
  moment_const_->setMinimum(0.);
  moment_const_->setValue(0.06);
  moment_const_->setSuffix(" m");
  addWidget(moment_const_);

  drag_const_ = new ParamGetterWidget_LinearEquation("Drag Constant", "", "CH", "φ");  // TODO
  drag_const_->setDecimals(6);
  drag_const_->setValue({ 0.01, 0.01 });
  drag_const_->setSuffix(" kg/rad");
  addWidget(drag_const_);
}

const char* AerodynamicsWidget_Manual::name() const
{
  return "Set Manually";
}

const char* AerodynamicsWidget_Manual::description() const
{
  return "Directly set the propeller aerodynamic constants.";
}

bool AerodynamicsWidget_Manual::isValid()
{
  return true;
}

void AerodynamicsWidget_Manual::copyFrom(const AerodynamicsWidget_Base* src)
{
  const auto derived = qt::qConstPointerCast<AerodynamicsWidget_Manual>(src);
  motor_const_->setValue(derived->motor_const_->getValue());
  moment_const_->setValue(derived->moment_const_->getValue());
  drag_const_->setValue(derived->drag_const_->getValue());
}

YAML::Node AerodynamicsWidget_Manual::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[motor_const_->name()] = motor_const_->getValue();
  node[moment_const_->name()] = moment_const_->getValue();
  node[drag_const_->name()] = drag_const_->getValue();

  return node;
}

void AerodynamicsWidget_Manual::load(const YAML::Node& node)
{
  motor_const_->setValue(node[motor_const_->name()].as<std::pair<double, double>>());
  moment_const_->setValue(node[moment_const_->name()].as<double>());
  drag_const_->setValue(node[drag_const_->name()].as<std::pair<double, double>>());
}

std::pair<double, double> AerodynamicsWidget_Manual::motorConst() const
{
  return motor_const_->getValue();
}

double AerodynamicsWidget_Manual::momentConst() const
{
  return moment_const_->getValue();
}

std::pair<double, double> AerodynamicsWidget_Manual::dragConst() const
{
  return drag_const_->getValue();
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
